Import("env")

# PlatformIO's build_flags reaches CCFLAGS/CXXFLAGS but not the final link line for
# a plain-driver flag like --coverage, so the coverage runtime never gets linked in.
# Append it to LINKFLAGS explicitly for the coverage env.
env.Append(LINKFLAGS=["--coverage"])
