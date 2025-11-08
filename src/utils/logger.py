"""
Logging utilities for the robot project.
"""

import logging
import sys
from pathlib import Path
from datetime import datetime


def setup_logger(name: str, log_file: str = None, level: int = logging.INFO) -> logging.Logger:
    """
    Setup logger with file and console handlers.
    
    Args:
        name: Logger name
        log_file: Path to log file (None for console only)
        level: Logging level
    
    Returns:
        Configured logger
    """
    logger = logging.getLogger(name)
    logger.setLevel(level)
    
    # Remove existing handlers
    logger.handlers.clear()
    
    # Console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(level)
    console_formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    console_handler.setFormatter(console_formatter)
    logger.addHandler(console_handler)
    
    # File handler (if log file specified)
    if log_file:
        log_path = Path(log_file)
        log_path.parent.mkdir(parents=True, exist_ok=True)
        
        file_handler = logging.FileHandler(log_file)
        file_handler.setLevel(level)
        file_formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        file_handler.setFormatter(file_formatter)
        logger.addHandler(file_handler)
    
    return logger


def get_logger(name: str) -> logging.Logger:
    """
    Get existing logger or create new one.
    
    Args:
        name: Logger name
    
    Returns:
        Logger instance
    """
    logger = logging.getLogger(name)
    if not logger.handlers:
        # Setup default logger if not configured
        return setup_logger(name)
    return logger

