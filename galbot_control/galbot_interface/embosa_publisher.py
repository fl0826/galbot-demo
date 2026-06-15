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

NODE_NAME = """ vla_python_publisher """

# Publisher class
class EmbosaPublisher:
    def __init__(self, topic):
        self.topic = topic
        self.index_ = 0

        embosa_python.EmbosaInit()
        self.qos = embosa_python.Qos()
        self.node = embosa_python.CreateNode(NODE_NAME)

        try:
            self.writer = self.node.CreateWriter(self.topic, self.qos)

            if self.writer is None:
                # 如果返回 None，表示操作失败，且没有抛出异常
                logger.error(NODE_NAME + "Failed to create entity.")
            else:
                logger.info(NODE_NAME + "Entity created successfully.")
        except RuntimeError as e:
            # 捕获并处理异常
            logger.error(NODE_NAME + f"Error occurred while creating entity: {e}")
        except Exception as e:
            # 捕获其他异常
            logger.error(NODE_NAME + f"An unexpected error occurred: {e}")

    def pub_mat(self,mat):
        # 发布数据

        message = struct.pack(
            "<iii",
            self.index_, 
            mat.shape[0],
            mat.shape[1]
        )

        message += mat.astype('<f4', copy=False).tobytes()

        if not self.writer.Publish(message):
            logger.error(f"SENT failed, index: {self.index_}")
            # i -= 1  # 重试
        
        self.index_ += 1

if __name__ == "__main__":
    publisher = EmbosaPublisher('embosa_vla_command')
    for i in range(100):
        data = np.array([[1.11,2.22],[-3.33,-44.4]])
        publisher.pub_mat(data)
        print(data)
        time.sleep(0.1)

