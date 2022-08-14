#ifndef TPLP_UI_TPLPADAPTER_H_
#define TPLP_UI_TPLPADAPTER_H_

namespace tplp {
// Provides a generic interface into whatever bits of the program the UI needs to work with. This serves as a dependency injection mechanism, allowing the same UI code to run both under the simulator and on the actual hardware.
class TplpAdapter {
    public:
};
} //

#endif  // TPLP_UI_TPLPADAPTER_H_