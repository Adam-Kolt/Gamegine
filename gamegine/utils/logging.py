import logging

logger = logging.getLogger("gamegine")
logging.basicConfig(level=logging.DEBUG)

# TODO: Add custom exceptions which when raised log to logger while also throwing descriptive error


def GetLogger():
    return logger


def Warn(message):
    logger.warning(message)


def Info(message):
    logger.info(message)


def Debug(message):
    logger.debug(message)


def Error(message):
    logger.error(message)


def Critical(message):
    logger.critical(message)


def SetLoggingLevel(level):
    logger.setLevel(level)
