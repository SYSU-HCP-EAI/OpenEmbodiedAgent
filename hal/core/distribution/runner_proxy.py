class RunnerProxy:
    def __init__(self, runner, is_remote):
        self.runner = runner
        self.is_remote = is_remote

    def __getattr__(self, name):
        def wrapper(*args, **kwargs):
            if self.is_remote:
                method = getattr(self.runner, name).remote
                return method(*args, **kwargs)
            else:
                method = getattr(self.runner, name)
                return method(*args, **kwargs)

        return wrapper
