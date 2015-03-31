#include "libroad_common.hpp"
#ifdef HAVE_CONFIG_H
const char *libroad_package_string()
{
    static const char package_string[] = PACKAGE_NAME " " PACKAGE_VERSION "-" GIT_VERSION " configured on " HOSTNAME " at " BUILD_DATE "\nConfigured with" CONFIGURE_ARGS;
    return package_string;
}
#else
const char *libroad_package_string()
{
    static const char package_string[] = "No config.h available";
    return package_string;
}
#endif


