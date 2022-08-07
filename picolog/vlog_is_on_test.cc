
#undef PICOLOG_VMODULE
#define PICOLOG_VMODULE "foo=3,vlog_is_on_test=2,bar=0"

#include "picolog/vlog_is_on.h"

namespace picolog {
namespace detail {

static_assert(ParseInt("0") == 0);
static_assert(ParseInt("15") == 15);
static_assert(ParseInt("999") == 999);

static_assert(ParseVmoduleConfig_1("") == VmoduleConfig_1{"", 0});
static_assert(ParseVmoduleConfig_1("a=1") == VmoduleConfig_1{"a", 1});
static_assert(ParseVmoduleConfig_1("Foo Bar=999") ==
              VmoduleConfig_1{"Foo Bar", 999});
static_assert(ParseVmoduleConfig_1("\\_a.= 1a5 ") ==
              VmoduleConfig_1{"\\_a.", 15});

static_assert(FilenameToModuleName("hello.cc") == "hello");
static_assert(FilenameToModuleName("hello.h") == "hello");
static_assert(FilenameToModuleName("/absolute/path/hello.cc") == "hello");
static_assert(FilenameToModuleName("relative/path/hello.cc") == "hello");
static_assert(FilenameToModuleName("C:\\windows\\path\\hello.cc") == "hello");

constexpr bool Config1Matches(const VmoduleConfig_1& config,
                              std::string_view module, int verboselevel) {
  return config.module == module && config.verboselevel == verboselevel;
}

constexpr VmoduleConfig actual =
    ParseVmoduleConfig("foo=3,picolog_test=2,bar=0");

static_assert(Config1Matches(actual[0], "foo", 3));
static_assert(Config1Matches(actual[1], "picolog_test", 2));
static_assert(Config1Matches(actual[2], "bar", 0));

// Using the PICOLOG_VMODULE value defined at the top of this file:
static_assert(VlogIsOnImpl("/path/to/foo.cc", 3));
static_assert(!VlogIsOnImpl("/path/to/foo.cc", 4));

// module=0 should disable all VLOG regardless of verboselevel
static_assert(!VlogIsOnImpl("/path/to/bar.cc", 0));
static_assert(!VlogIsOnImpl("/path/to/bar.cc", -1));

}  // namespace detail
}  // namespace picolog

// Using the PICOLOG_VMODULE value defined at the top of this file:
static_assert(VLOG_IS_ON(1));
static_assert(VLOG_IS_ON(2));
static_assert(!VLOG_IS_ON(3));
static_assert(!VLOG_IS_ON(4));

int main() {
  return 0;
  // Make sure disabled VLOG() levels are not compiled at all, by testing for
  // side effects.
  // TODO: this belongs in picolog_test.cc, which isn't a thing yet
  //       because of the hard dependency on pico-sdk preventing us from
  //       building & running this on the host.
  //
  // int ret = 0
  // VLOG(3) << "Shouldn't happen: " << (ret = 1);
  // return ret;
}