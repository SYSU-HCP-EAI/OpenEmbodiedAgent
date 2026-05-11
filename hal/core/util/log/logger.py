import logging


class Logger(object):
    """global logger

    Args:
        filename (str, optional): log file name. Defaults to None.
        level (str, optional): log level( debug info warning error critical ). Defaults to 'info'.
        fmt (str, optional): log format. Defaults to '[%(asctime)s][%(levelname)s] %(message)s'.
    PS:
        more format details at : https://docs.python.org/zh-cn/3/library/logging.html
    """

    level_relations = {
        'debug': logging.DEBUG,
        'info': logging.INFO,
        'warning': logging.WARNING,
        'error': logging.ERROR,
        'critical': logging.CRITICAL,
    }

    # '[%(asctime)s][%(levelname)s] %(pathname)s[line:%(lineno)d] -: %(message)s'
    def __init__(
        self, filename: str = None, level: str = 'info', fmt: str = '[%(asctime)s][%(levelname)s] %(message)s'
    ):
        if filename == 'None':
            filename = None
        self.log = logging.getLogger(filename)
        format_str = logging.Formatter(fmt)
        self.log.setLevel(self.level_relations.get(level))
        sh = logging.StreamHandler()
        sh.setFormatter(format_str)
        self.log.addHandler(sh)
        # Logging file
        if filename is not None:
            th = logging.FileHandler(filename=filename, encoding='utf-8')
            th.setFormatter(format_str)
            self.log.addHandler(th)
