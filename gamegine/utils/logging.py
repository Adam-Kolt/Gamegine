import logging

logger = logging.getLogger("gamegine")
logging.basicConfig(level=logging.CRITICAL)

# TODO: Add custom exceptions which when raised log to logger while also throwing descriptive error


def GetLogger():
    """Returns the logger object used by the gamegine library.

    :return: The logger object used by the gamegine library.
    :rtype: logging.Logger
    """
    return logger


def Warn(message: str):
    """Logs a warning message.

    :param message: The message to log.
    :type message: str
    """
    logger.warning(message)


def Info(message: str):
    """Logs an informational message.

    :param message: The message to log.
    :type message: str
    """
    logger.info(message)


def Debug(message: str):
    """Logs a debug message.

    :param message: The message to log.
    :type message: str
    """
    logger.debug(message)


def Error(message: str):
    """Logs an error message.

    :param message: The message to log.
    :type message: str
    """
    logger.error(message)


def Critical(message: str):
    """Logs a critical message.

    :param message: The message to log.
    :type message: str
    """
    logger.critical(message)


def SetLoggingLevel(level):
    """Sets the logging level of the logger.

    :param level: The logging level to set.
    :type level: int
    """
    logger.setLevel(level)
