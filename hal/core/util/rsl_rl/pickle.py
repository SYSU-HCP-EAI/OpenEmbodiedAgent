import pickle


class Unpickler(pickle.Unpickler):
    def find_class(self, module, name):
        if name == 'Normalizer' and module == 'rsl_rl.utils.utils':
            from internutopia.core.util.rsl_rl.normalizer import Normalizer

            return Normalizer
        return super().find_class(module, name)
