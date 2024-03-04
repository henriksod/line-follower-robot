def _build_variant_transition_impl(settings, attr):
    if settings["//bazel/flags/agents:agents"] != "simulation":
        fail("Overwriting explicitly set variant {} with {}".format(
            settings["//bazel/flags/agents:agents"],
            attr.variant,
        ))

    return {
        "//command_line_option:compilation_mode": attr.compilation_mode,
        "//command_line_option:copt": attr.copts,
        "//bazel/flags/agents": attr.variant,
    }

_build_variant_transition = transition(
    implementation = _build_variant_transition_impl,
    inputs = [
        "//bazel/flags/agents:agents",
        "//command_line_option:copt",
    ],
    outputs = [
        "//bazel/flags/agents",
        "//command_line_option:compilation_mode",
        "//command_line_option:copt",
    ],
)

def _build_variant_impl(ctx):
    runfiles_depsets = []
    files_depsets = []
    cc_infos = []
    for target in ctx.attr.targets:
        runfiles_depsets.append(target[DefaultInfo].default_runfiles.files)
        files_depsets.append(target[DefaultInfo].files)
        if CcInfo in target:
            cc_infos.append(target[CcInfo])

    return [
        DefaultInfo(
            files = depset(transitive = files_depsets),
            runfiles = ctx.runfiles(
                transitive_files = depset(transitive = runfiles_depsets),
            ),
        ),
        cc_common.merge_cc_infos(cc_infos = cc_infos),
    ]

_build_variant = rule(
    implementation = _build_variant_impl,
    attrs = {
        "compilation_mode": attr.string(default = "fastbuild"),
        "copts": attr.string_list(),
        "targets": attr.label_list(cfg = _build_variant_transition),
        "variant": attr.string(
            doc = "Available variants are listed at //bazel/flags/agents",
        ),
    },
)

def build_variant(**kwargs):
    _build_variant(**kwargs)
