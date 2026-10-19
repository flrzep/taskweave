"""Per-node singletons: dropped on destroy_node, never served to another node that reuses an id."""

from taskweave_core.node_cache import for_node


class FakeNode:
    def __init__(self):
        self.destroyed = 0

    def destroy_node(self):
        self.destroyed += 1


class Thing:
    def __init__(self, node):
        self.node = node


def test_same_node_same_instance_and_destroy_drops_it():
    cache, node = {}, FakeNode()
    first = for_node(cache, node, Thing)
    assert for_node(cache, node, Thing) is first
    node.destroy_node()
    assert node.destroyed == 1 and cache == {}


def test_a_stale_entry_of_a_reused_id_is_replaced():
    cache, node = {}, FakeNode()
    cache[id(node)] = Thing(FakeNode())          # left by a dead node whose id this one got
    assert for_node(cache, node, Thing).node is node
