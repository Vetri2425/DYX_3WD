from setuptools import setup

package_name = "dyx3_rpp_legacy"

# QUARANTINED — the single permitted exception to "no Python in the control graph".
# Terms (architecture section 7.5):
#   - never installed by install.sh --production
#   - excluded from the release artifact
#   - no other package may depend on it
#   - DELETED when Gate 7 (full-mission equivalence) passes
# CI job `legacy_quarantine` enforces all four.
setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Vetri",
    maintainer_email="vetri96anitha@gmail.com",
    description="Quarantined Python RPP shadow oracle. Deleted at Gate 7.",
    license="Proprietary",
    entry_points={"console_scripts": []},
)
