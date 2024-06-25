class NamedObject(object):
    def __init__(self, name: str) -> None:
        self.name = name
        pass

    def prefix(self, prefix: str) -> 'NamedObject':
        self.name = prefix + self.name
        return self
    
    def suffix(self, suffix: str) -> 'NamedObject':
        self.name = self.name + suffix
        return self
    
