from threading import Event

############################
# OrEvent - by https://stackoverflow.com/a/12320352

def new_set(e, old_set):
    old_set()
    e.changed()

def new_clear(e, old_clear):
    old_clear()
    e.changed()

def orify(e, changed_callback):
    old_set = e.set
    old_clear = e.clear
    e.changed = changed_callback
    e.set = lambda: new_set(e, old_set)
    e.clear = lambda: new_clear(e, old_clear)

def OrEvent(*events):
    or_event = Event()
    def changed():
        bools = [e.is_set() for e in events]
        if any(bools):
            or_event.set()
        else:
            or_event.clear()
    for e in events:
        orify(e, changed)
    changed()
    return or_event