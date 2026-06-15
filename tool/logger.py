import logging
import os
import sys
import time
from datetime import datetime
from logging.handlers import TimedRotatingFileHandler, QueueHandler, QueueListener
from colorama import Fore, Style
import queue
import threading

class LoggerManager:
    """Async Logger Manager with QueueHandler + QueueListener."""

    LOG_COLORS = {
        "DEBUG": Fore.BLUE,
        "INFO": Fore.GREEN,
        "WARNING": Fore.YELLOW,
        "ERROR": Fore.RED,
        "CRITICAL": Fore.RED + Style.BRIGHT,
    }

    _loggers = {}
    _last_check_date = {}
    _queue_listeners = {}

    @staticmethod
    def _archive_old_log_if_needed(log_file_path):
        """Check if log file exists and contains logs from previous days. If so, archive them.

        Args:
            log_file_path (str): Full path to the log file
        """
        if not os.path.exists(log_file_path):
            return  # No existing log file, nothing to archive

        try:
            today = datetime.now().date()
            log_dir = os.path.dirname(log_file_path)
            log_basename = os.path.basename(log_file_path)
            log_name_without_ext = os.path.splitext(log_basename)[0]

            # Read the log file and separate entries by date
            with open(log_file_path, "r", encoding="utf-8") as log_file:
                lines = log_file.readlines()

            if not lines:
                return  # Empty log file

            # Group log lines by date
            logs_by_date = {}
            current_date = None

            for line in lines:
                # Try to extract date from log line format: [YYYY-MM-DD HH:MM:SS.mmm]
                if line.startswith("[") and len(line) > 23:
                    try:
                        date_str = line[1:11]  # Extract YYYY-MM-DD
                        line_date = datetime.strptime(date_str, "%Y-%m-%d").date()
                        current_date = line_date
                    except (ValueError, IndexError):
                        pass  # Not a valid date format, use current_date

                # Group line by date (use today if no date found)
                if current_date is None:
                    current_date = today

                if current_date not in logs_by_date:
                    logs_by_date[current_date] = []
                logs_by_date[current_date].append(line)

            # Archive logs from previous days
            old_logs_found = False
            today_logs = []

            for log_date, log_lines in logs_by_date.items():
                if log_date < today:
                    old_logs_found = True
                    # Archive these logs with consistent naming format
                    # Format: log_name_YYYY_MM_DD.log (consistent with TimedRotatingFileHandler)
                    archive_name = f"{log_name_without_ext}_{log_date.strftime('%Y_%m_%d')}.log"
                    archive_path = os.path.join(log_dir, archive_name)

                    with open(archive_path, "a", encoding="utf-8") as archive_file:
                        archive_file.writelines(log_lines)
                    print(f"Archived logs from {log_date} to: {archive_path}")
                else:
                    # Keep today's logs
                    today_logs.extend(log_lines)

            # If we archived old logs, rewrite the current log file with only today's logs
            if old_logs_found:
                with open(log_file_path, "w", encoding="utf-8") as log_file:
                    log_file.writelines(today_logs)
                print(f"Cleaned log file, kept {len(today_logs)} lines from today")

        except Exception as e:
            print(f"Warning: Failed to archive old log file {log_file_path}: {e}")

    @classmethod
    def get_logger(cls, log_name="vla_logger", log_dir="/userdata/log/galbot_mobile_manipulation", retain_days=7):
        """Get async logger."""
        if log_name not in cls._loggers:
            logger = logging.getLogger(log_name)
            logger.setLevel(logging.DEBUG)
            logger.propagate = False

            os.makedirs(log_dir, exist_ok=True)
            log_file_path = os.path.join(log_dir, f"{log_name}.log")
            cls._archive_old_log_if_needed(log_file_path)

            # --------------------
            # 创建队列 & QueueHandler
            log_queue = queue.Queue(-1)  # 无限队列
            queue_handler = QueueHandler(log_queue)
            logger.addHandler(queue_handler)

            # --------------------
            # 创建真正写入文件和终端的 handlers
            console_handler = logging.StreamHandler(sys.stdout)
            console_handler.setLevel(logging.DEBUG)
            console_handler.setFormatter(cls.CustomFormatter())

            file_handler = TimedRotatingFileHandler(
                log_file_path, when="midnight", interval=1, backupCount=retain_days, encoding="utf-8"
            )
            file_handler.namer = lambda name: name.replace(".log", "") + ".log"
            file_handler.suffix = "_%Y_%m_%d.log"
            file_handler.setLevel(logging.DEBUG)
            file_handler.setFormatter(
                logging.Formatter(
                    "[%(asctime)s.%(msecs)03d] [%(levelname)s] [%(filename)s: %(funcName)s: %(lineno)d]  %(message)s",
                    "%Y-%m-%d %H:%M:%S",
                )
            )
            file_handler.addFilter(cls.DateChangeFilter(log_file_path, log_name))

            # --------------------
            # QueueListener 异步写日志
            listener = QueueListener(log_queue, console_handler, file_handler, respect_handler_level=True)
            listener.start()

            cls._queue_listeners[log_name] = listener
            cls._loggers[log_name] = logger
            cls._last_check_date[log_name] = datetime.now().date()

        return cls._loggers[log_name]

    # --------------------
    class DateChangeFilter(logging.Filter):
        def __init__(self, log_file_path, log_name):
            super().__init__()
            self.log_file_path = log_file_path
            self.log_name = log_name

        def filter(self, record):
            current_date = datetime.now().date()
            if self.log_name in LoggerManager._last_check_date:
                last_date = LoggerManager._last_check_date[self.log_name]
                if current_date > last_date:
                    LoggerManager._archive_old_log_if_needed(self.log_file_path)
                    LoggerManager._last_check_date[self.log_name] = current_date
            return True

    # --------------------
    class CustomFormatter(logging.Formatter):
        def format(self, record):
            log_color = LoggerManager.LOG_COLORS.get(record.levelname, "")
            log_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(record.created))
            log_milliseconds = f"{int(record.msecs):03d}"
            log_header = (
                f"{log_color}[{log_time}.{log_milliseconds}] "
                f"[{record.levelname}] "
                f"[{record.filename}: {record.funcName}: {record.lineno}]: {Style.RESET_ALL}"
            )
            log_message = f"\n {record.getMessage()}"
            return log_header + log_message

    @classmethod
    def stop_listener(cls, log_name="vla_logger"):
        """Stop the QueueListener (call at program exit)"""
        if log_name in cls._queue_listeners:
            cls._queue_listeners[log_name].stop()


# Pick up the log directory from environment when running under systemd.
# Priority: ENV variable > deployment path > local logs directory
DEFAULT_LOG_DIR = os.environ.get("LOG_DIR", "logs")

try:
    logger = LoggerManager.get_logger(retain_days=7, log_dir=DEFAULT_LOG_DIR)
except PermissionError:
    fallback_dir = os.path.join(os.getcwd(), "logs")
    logger = LoggerManager.get_logger(retain_days=7, log_dir=fallback_dir)

if __name__ == "__main__":
    logger.debug("Debug message")
    logger.info("System running normally")
    logger.warning("Warning detected")
    logger.exception("Exception occurred")
    logger.error("Error occurred")
    logger.critical("Critical failure!")
