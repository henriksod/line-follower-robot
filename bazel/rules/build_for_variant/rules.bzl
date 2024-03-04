load("//bazel/rules/transitions:defs.bzl", "build_variant")

def _add_transition_variant(variant, kwargs):
    label = kwargs["name"] + "_variant_" + variant
    build_variant(
        name = label,
        targets = kwargs["deps"],
        tags = ["manual"],
        variant = variant,
    )
    kwargs["deps"] = [":" + label]
    return kwargs

def cc_binary_with_variant(variant, **kwargs):
    """Add a cc_binary with specific variant of agents.

    Args:
        variant:    kwargs["deps"] will transition to use //bazel/flags/agents=<agents>
        **kwargs:   all arguments to cc_binary are accepted
    """

    kwargs = _add_transition_variant(variant, kwargs)

    native.cc_binary(
        **kwargs
    )
