import os
import argparse
from threading import Condition

import time
import struct
from dataclasses import dataclass
from typing import List

import embosa_extend_node
import embosa_python

import numpy as np
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from tool.logger import logger, LoggerManager


NODE_NAME = """ vla_python_client """

# Client class
class EmbosaClient:
    def __init__(self, topic):
        self.topic = topic
        self.index_ = 0

        embosa_python.EmbosaInit()
        self.qos = embosa_python.RpcQos()
        self.node = embosa_python.CreateNode(NODE_NAME)

        try:
            self.client = self.node.CreateClient(self.topic, self.qos)

            if self.client is None:
                # 如果返回 None，表示操作失败，且没有抛出异常
                logger.error(NODE_NAME + "Failed to create entity.")
            else:
                logger.info(NODE_NAME + "WaitForServerConnected.")
                self.client.WaitForServerConnected()
                logger.info(NODE_NAME + "ServerConnected.")
        except RuntimeError as e:
            # 捕获并处理异常
            logger.error(NODE_NAME + f"Error occurred while creating entity: {e}")
        except Exception as e:
            # 捕获其他异常
            logger.error(NODE_NAME + f"An unexpected error occurred: {e}")

    def request_service(self,data_string):
        
        buf_in = data_string.encode('utf-8') 
        buf_out = bytearray(128)  # 输出缓冲区

        if self.client.SendRequestWrapper(buf_in, buf_out) == True:
            return buf_out.decode('utf-8').strip('\x00')
        else:
            return "请求失败"

if __name__ == "__main__":
    client = EmbosaClient('embosa_vla_service')
    print(client.request_service("ok"))
    

