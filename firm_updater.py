#!/usr/bin/python3
import os
import socket
import time
import struct
# import construct as cstruct

FILENAME = "embdAI.bin"
SERVER_PORT = 58114
CHUNK_SIZE = 512
format_string = ">" + str(CHUNK_SIZE) + "c"
# format_string = ">c"

ACK_MESSAGE = "ack_packet"
END_MESSAGE = "end_transmision"

def get_file_size(filename):
    st = os.stat(filename)
    return st.st_size

def start_update():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(('', SERVER_PORT))
        s.listen()
        conn, addr = s.accept()
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        with conn:
            print("Connected by: ", addr)
            file = open(FILENAME, 'rb')
            file_size = get_file_size(FILENAME)
            amount_of_chunks = file_size // CHUNK_SIZE + (file_size % CHUNK_SIZE > 0)
            print('File size is {} or {} chunks'.format(file_size, amount_of_chunks))

            data = file.read(CHUNK_SIZE)
            output_data = data

            chunks = 1
            while output_data:
                in_data = conn.recv(256)
                if ACK_MESSAGE in str(in_data):
                    print('Sending {}/{}...'.format(chunks,amount_of_chunks))
                    conn.send(output_data)
                    data = file.read(CHUNK_SIZE)
                    output_data = data
                    chunks+=1
                elif END_MESSAGE in str(in_data):
                    break
            print("Firmware upgraded")
            # time.sleep(2)
            conn.shutdown(socket.SHUT_WR)
            conn.close()
            file.close()
    s.close()


# start_update()