
# ============================================== #
#   enumerates which launch configurations are 
#   associated with each mode in the graph.
#   Typically, configurations specify which
#   nodes are too be launch in a particular mode
# ============================================== #


MODES = {
    "full" : {

    },
    "sim"  : {},
    "surface" : {},
    "demo" : {},
    "sensors" : {},

    "core" : {},


    "no_bridge": {
        "depth_converter_on": False,
        "dvl_converter_on": False,
        "dvl_global_on": False,
        "seatrac_ahrs_converter_on": False
    }
}

COMPOUND_FLAGS = {
    "bridge": {
        "depth_converter_on": True,
        "dvl_converter_on": True,
        "dvl_global_on": True,
        "seatrac_ahrs_converter_on": True
    }
}

DEFAULTS = {

}





def resolve_modes(mode_list, flag_overrides):
    """
    mode_list: ["manual", "sim"]
    flag_overrides: {"use_navigation": True, "use_gps": False}

    Returns dict of launch configs to set.
    """

    resolved = dict(DEFAULTS)

    # update a mode with mode inheritance
    def update_mode(resolved:dict, mode:str, visited:set=None):
        if mode not in MODES:
            raise RuntimeError(f"Unknown mode: {mode}")
        if visited is None: visited = {}
        # update parents first so child mode overrides
        for parent in MODES[mode]['__inherits__']:
            if parent not in visited:
                visited.add(parent)
                update_mode(resolved, parent, visited)
        resolved.update(MODES[mode])
        del resolved['__inherits__'] # don't include special __inherits__ in final

    # Apply modes from lowest → highest priority
    # first mode wins → so apply reversed
    for mode in reversed(mode_list):
        update_mode(resolved, mode)
        if mode not in MODES:
            raise RuntimeError(f"Unknown mode: {mode}")
        resolved.update(MODES[mode])

    # Explicit flags override everything
    resolved.update(flag_overrides)

    return resolved



def add_compound_flag(resolved:dict, flag:str, value:bool):
    # add subflags if flag is compound. Otherwise, add subvalue directly
    if flag in COMPOUND_FLAGS:
        for subflag, subvalue in COMPOUND_FLAGS[flag].items():
            # if compound flag value = False, invert subvalue
            add_compound_flag(subflag, value==subvalue)
    else:
        resolved[flag] = value



def parse_list(value: str):
    if not value:
        return []
    return [x.strip() for x in value.split(",") if x.strip()]


def parse_flags(flag_str: str):
    """
    use_gps:false,use_navigation:true, verbosity_level:5
    """
    result = {}
    if not flag_str:
        return result

    for item in flag_str.split(","):
        item = item.strip()
        if not item:
            continue

        if ":" in item:
            k, v = item.split(":", 1)
            if v.strip().lower() == "true":
                result[k.strip()] = True
            elif v.strip().lower() == "false":
                result[k.strip()] = False
            else: 
                result[k.strip()] = v.strip()
        else:
            # shorthand → true
            result[item] = True

    return result