"""TF buffer that survives a simulation restart: ``buffer(node)`` returns a Buffer and TransformListener that the
node's ``clock_jump`` watcher clears when sim time goes back (see clock_jump.py for why)."""

from __future__ import annotations


def buffer(node, jump_s: float = 0.5, settle_s: float = 2.0):
    """(Buffer, TransformListener) for ``node``; keep both referenced as long as they are used."""
    from tf2_ros import Buffer, TransformListener

    from . import clock_jump

    tf = Buffer()
    listener = TransformListener(tf, node)
    clock_jump.watch(node, jump_s, settle_s).add_buffer(tf)
    return tf, listener
