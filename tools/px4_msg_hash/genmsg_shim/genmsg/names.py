def package_resource_name(name):
    if "/" in name:
        pkg, res = name.split("/", 1)
        return pkg, res
    return "", name
