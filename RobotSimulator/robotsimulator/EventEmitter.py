class EventEmitter:
    def __init__(self):
        self._subscribers = []

    def __iadd__(self, method):
        self._subscribers.append(method)
        return self

    def __isub__(self, method):
        self._subscribers.remove(method)
        return self
    
    def emit(self, *args, **keywargs):
        for method in self._subscribers:
            method(*args, **keywargs)