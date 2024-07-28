import logging

logger = logging.getLogger("gamegine")
logging.basicConfig(level=logging.DEBUG)


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
