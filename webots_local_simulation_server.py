#!/usr/bin/env python3

# Based on local_simulation_server.py found here https://github.com/cyberbotics/webots-server/tree/main

# Copyright 1996-2022 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Webots local simulation server."""

import os
import socket
import subprocess
import sys
import re

HOST = ''  # Any host can connect
PORT = 2000 if len(sys.argv) < 2 else int(sys.argv[1])  # Port to listen on


def close_connection(connection, message):
    connection.sendall(message.encode('utf-8'))
    print(message, file=sys.stderr)
    connection.close()


tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

tcp_socket.bind((HOST, PORT))
tcp_socket.listen()
while True:
    print(f'Waiting for connection on port {PORT}...')
    connection, address = tcp_socket.accept()

    print(f'Connection from {address}')
    data = connection.recv(1024)
    command = data.decode('utf-8').split(' ')

    if not command[0].endswith('webots'):
        message = f'FAIL: \'{command[0]}\' is not recognized as a Webots executable.'
        close_connection(connection, message)
        continue
    else:
        if not os.path.isabs(command[0]) and command[0] != 'webots':
            message = f'FAIL: \'{command[0]}\' must be either \'webots\' or an absolute path to the executable.'
            close_connection(connection, message)
            continue

    if os.path.isabs(command[0]):
        pass
    elif 'WEBOTS_HOME' in os.environ:
        if sys.platform == 'darwin':
            # macOS
            path_suffix = 'Contents/MacOS/webots'
        elif sys.platform == 'linux':
            # Linux
            path_suffix = 'webots'
        else:
            # Windows
            path_suffix = 'msys64\mingw64\bin\webots.exe'
        command[0] = os.path.join(os.environ['WEBOTS_HOME'], path_suffix)
    else:
        message = 'FAIL: WEBOTS_HOME environment variable is not defined. Please define a valid Webots installation folder.'
        close_connection(connection, message)
        continue

    invalid_world_file = False
    for idx, argument in enumerate(command[1:]):
        if not argument.startswith('/'):
            continue

        # +1 and +2 instead of +0 and +1 because we start from 1 instead of 0
        world_file = ' '.join(command[idx+1:]).replace("\\ ", " ")
        if not world_file.endswith('.wbt'):
            message = f'FAIL: Incorrect world file \'{world_file}\'.'
            close_connection(connection, message)
            invalid_world_file = True
            break

        world_file = re.sub(r"/mnt/([a-zA-Z])", r"\g<1>:", world_file)

        if not os.path.isfile(world_file):
            message = f'FAIL: The world file \'{world_file}\' doesn\'t exist.'
            close_connection(connection, message)
            invalid_world_file = True
            break

        command[idx+1] = '"' + world_file + '"'
        command = command[:idx+2]
    if invalid_world_file:
        continue

    try:
        webots_process = subprocess.Popen(command)
    except FileNotFoundError:
        message = f'FAIL: \'{command[0]}\' could not be found on the host.'
        close_connection(connection, message)
        continue

    connection.sendall(b'ACK')
    connection.settimeout(1)
    connection_closed = False
    while webots_process.poll() is None:
        try:
            data = connection.recv(1024)
        except socket.timeout:
            continue
        else:
            if not data:
                print('Connection was closed by the client.')
                connection.close()
                webots_process.kill()
                connection_closed = True
                break

    if connection_closed:
        connection_closed = False
        continue

    print('Webots was executed successfully.')
    closing_message = 'CLOSED'
    connection.sendall(closing_message.encode('utf-8'))
    connection.close()
