import logging
from datetime import datetime
from pathlib import Path

import colorlog

class ProjectLogger:
    _logger = None
    _verbose = False

    @classmethod
    def setup_logger(
        cls,
        verbose: bool = False,
        log_to_file: bool = True,
        log_dir: str = "logs",
        timestamp: str | None = None,
    ) -> logging.Logger:
        cls._verbose = verbose

        if cls._logger is not None:
            if verbose:
                cls._logger.setLevel(logging.DEBUG)
            else:
                cls._logger.setLevel(logging.INFO)

            if log_to_file:
                for handler in cls._logger.handlers[:]:
                    if isinstance(handler, logging.FileHandler):
                        handler.close()
                        cls._logger.removeHandler(handler)

                try:
                    log_path = Path(log_dir)
                    log_path.mkdir(parents=True, exist_ok=True)

                    log_filename = "navigation_benchmark.log"

                    file_handler = logging.FileHandler(log_path / log_filename)
                    file_formatter = logging.Formatter(
                        "%(asctime)s - %(name)s - %(levelname)s - %(funcName)s:%(lineno)d - %(message)s",
                        datefmt="%Y-%m-%d %H:%M:%S",
                    )
                    file_handler.setFormatter(file_formatter)
                    cls._logger.addHandler(file_handler)
                except Exception as e:
                    cls._logger.warning(f"Could not setup file logging: {e}")

            return cls._logger

        cls._logger = logging.getLogger("Navigation Benchmark")
        cls._logger.setLevel(logging.DEBUG if verbose else logging.INFO)

        if cls._logger.handlers:
            cls._logger.handlers.clear()

        console_handler = colorlog.StreamHandler()
        console_formatter = colorlog.ColoredFormatter(
            "%(log_color)s%(asctime)s - %(name)s - %(levelname)s - %(message)s",
            datefmt="%H:%M:%S",
            log_colors={
                "DEBUG": "cyan",
                "INFO": "green",
                "WARNING": "yellow",
                "ERROR": "red",
                "CRITICAL": "red,bg_white",
            },
        )
        console_handler.setFormatter(console_formatter)
        cls._logger.addHandler(console_handler)

        if log_to_file:
            try:
                log_path = Path(log_dir)
                log_path.mkdir(parents=True, exist_ok=True)

                if timestamp is None:
                    log_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                else:
                    log_timestamp = timestamp
                log_filename = f"navigation_benchmark_{log_timestamp}.log"

                file_handler = logging.FileHandler(log_path / log_filename)
                file_formatter = logging.Formatter(
                    "%(asctime)s - %(name)s - %(levelname)s - %(funcName)s:%(lineno)d - %(message)s",
                    datefmt="%Y-%m-%d %H:%M:%S",
                )
                file_handler.setFormatter(file_formatter)
                cls._logger.addHandler(file_handler)
            except Exception as e:
                cls._logger.warning(f"Could not setup file logging: {e}")

        return cls._logger

    @classmethod
    def get_logger(cls, name: str | None = None) -> logging.Logger:
        if cls._logger is None:
            cls.setup_logger(log_to_file=False)

        if name:
            return cls._logger.getChild(name)
        return cls._logger

    @classmethod
    def is_verbose(cls) -> bool:
        return cls._verbose

def get_logger(name: str | None = None) -> logging.Logger:
    return ProjectLogger.get_logger(name)

def setup_logging(
    verbose: bool = False,
    log_to_file: bool = True,
    log_dir: str = "logs",
    timestamp: str | None = None,
) -> logging.Logger:
    return ProjectLogger.setup_logger(verbose, log_to_file, log_dir, timestamp)
