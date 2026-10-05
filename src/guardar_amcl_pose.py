#!/usr/bin/env python3

import math
import os
import select
import sys
import termios
import threading
import tty

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class AMCLPoseSaver(Node):

    def __init__(self):
        super().__init__('amcl_pose_saver')

        # Carpeta donde se guardarán los archivos:
        # /home/tu_usuario/Desktop/poses
        self.output_directory = os.path.expanduser('~/Desktop/poses')

        # Crea la carpeta automáticamente si no existe.
        os.makedirs(self.output_directory, exist_ok=True)

        # Guarda la última pose recibida desde /amcl_pose.
        self.latest_pose = None
        self.pose_lock = threading.Lock()

        # Busca el siguiente nombre disponible:
        # ws1.txt, ws2.txt, ws3.txt...
        self.waypoint_number = self.find_next_waypoint_number()

        # Suscripción al tópico de localización AMCL.
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

        self.get_logger().info('Suscripto al tópico /amcl_pose')
        self.get_logger().info("Presioná la tecla 'g' para guardar la pose.")
        self.get_logger().info("Presioná la tecla 'q' para cerrar el programa.")
        self.get_logger().info(
            f'Los archivos se guardarán en: {self.output_directory}'
        )

    def pose_callback(self, message):
        """
        Se ejecuta cada vez que llega un mensaje desde /amcl_pose.
        Conserva en memoria la última pose recibida.
        """

        with self.pose_lock:
            self.latest_pose = message

    def find_next_waypoint_number(self):
        """
        Busca el siguiente número disponible para no sobrescribir
        archivos existentes.
        """

        number = 1

        while True:
            filename = os.path.join(
                self.output_directory,
                f'ws{number}.txt'
            )

            if not os.path.exists(filename):
                return number

            number += 1

    @staticmethod
    def quaternion_to_euler(x, y, z, w):
        """
        Convierte un cuaternión en ángulos Euler:
        roll, pitch y yaw.

        Los resultados se devuelven en radianes.
        """

        # Roll: rotación sobre el eje X.
        sin_roll_cos_pitch = 2.0 * (w * x + y * z)
        cos_roll_cos_pitch = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(
            sin_roll_cos_pitch,
            cos_roll_cos_pitch
        )

        # Pitch: rotación sobre el eje Y.
        sin_pitch = 2.0 * (w * y - z * x)

        if abs(sin_pitch) >= 1.0:
            pitch = math.copysign(
                math.pi / 2.0,
                sin_pitch
            )
        else:
            pitch = math.asin(sin_pitch)

        # Yaw: rotación sobre el eje Z.
        sin_yaw_cos_pitch = 2.0 * (w * z + x * y)
        cos_yaw_cos_pitch = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(
            sin_yaw_cos_pitch,
            cos_yaw_cos_pitch
        )

        return roll, pitch, yaw

    def save_pose(self):
        """
        Guarda la última pose recibida en un archivo TXT.
        """

        with self.pose_lock:
            if self.latest_pose is None:
                self.get_logger().warning(
                    'Todavía no se recibió información desde /amcl_pose.'
                )
                return

            message = self.latest_pose

        position = message.pose.pose.position
        orientation = message.pose.pose.orientation

        roll, pitch, yaw = self.quaternion_to_euler(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        )

        filename = os.path.join(
            self.output_directory,
            f'ws{self.waypoint_number}.txt'
        )

        try:
            with open(filename, 'w', encoding='utf-8') as file:

                file.write(
                    f'waypoint: ws{self.waypoint_number}\n'
                )

                file.write(
                    f'frame_id: {message.header.frame_id}\n'
                )

                file.write('\n[position]\n')
                file.write(f'x: {position.x:.8f}\n')
                file.write(f'y: {position.y:.8f}\n')
                file.write(f'z: {position.z:.8f}\n')

                file.write('\n[orientation_quaternion]\n')
                file.write(f'x: {orientation.x:.8f}\n')
                file.write(f'y: {orientation.y:.8f}\n')
                file.write(f'z: {orientation.z:.8f}\n')
                file.write(f'w: {orientation.w:.8f}\n')

                file.write('\n[orientation_euler_radians]\n')
                file.write(f'roll: {roll:.8f}\n')
                file.write(f'pitch: {pitch:.8f}\n')
                file.write(f'yaw: {yaw:.8f}\n')

                file.write('\n[orientation_euler_degrees]\n')
                file.write(
                    f'roll: {math.degrees(roll):.4f}\n'
                )
                file.write(
                    f'pitch: {math.degrees(pitch):.4f}\n'
                )
                file.write(
                    f'yaw: {math.degrees(yaw):.4f}\n'
                )

            self.get_logger().info(
                f'Pose guardada correctamente en: {filename}'
            )

            # Incrementa el número para el siguiente archivo.
            self.waypoint_number += 1

        except OSError as error:
            self.get_logger().error(
                f'No se pudo guardar el archivo: {error}'
            )


def keyboard_loop(node):
    """
    Lee el teclado sin necesidad de presionar Enter.

    g: guarda la pose actual.
    q: finaliza el programa.
    """

    if not sys.stdin.isatty():
        node.get_logger().error(
            'El programa debe ejecutarse desde una terminal interactiva.'
        )

        if rclpy.ok():
            rclpy.shutdown()

        return

    terminal_settings = termios.tcgetattr(sys.stdin)

    try:
        # Permite leer una tecla directamente, sin presionar Enter.
        tty.setcbreak(sys.stdin.fileno())

        while rclpy.ok():
            readable, _, _ = select.select(
                [sys.stdin],
                [],
                [],
                0.1
            )

            if not readable:
                continue

            key = sys.stdin.read(1).lower()

            if key == 'g':
                node.save_pose()

            elif key == 'q':
                node.get_logger().info(
                    'Cerrando el programa.'
                )

                if rclpy.ok():
                    rclpy.shutdown()

                break

    except Exception as error:
        node.get_logger().error(
            f'Error al leer el teclado: {error}'
        )

        if rclpy.ok():
            rclpy.shutdown()

    finally:
        # Restaura la configuración normal de la terminal.
        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            terminal_settings
        )


def main(args=None):
    rclpy.init(args=args)

    node = AMCLPoseSaver()

    keyboard_thread = threading.Thread(
        target=keyboard_loop,
        args=(node,),
        daemon=True
    )

    keyboard_thread.start()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info(
            'Programa interrumpido con Ctrl+C.'
        )

    finally:
        if rclpy.ok():
            rclpy.shutdown()

        node.destroy_node()


if __name__ == '__main__':
    main()