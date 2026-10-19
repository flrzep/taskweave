"""One shared helper object per node (a class-level dict keyed by id(node)) that does not outlive the node."""

import threading

_lock = threading.RLock()


def for_node(cache: dict, node, make):
    """``cache[id(node)]``, made with ``make(node)`` when missing or left by another node of the same id. The entry
    goes when the node is destroyed (its value holds the node, so only this hook can free it)."""
    key = id(node)
    with _lock:
        item = cache.get(key)
        if item is None or item.node is not node:
            item = cache[key] = make(node)
            destroy = node.destroy_node

            def destroy_node(*args, **kwargs):
                with _lock:
                    if cache.get(key) is item:
                        del cache[key]
                return destroy(*args, **kwargs)
            node.destroy_node = destroy_node
        return item
