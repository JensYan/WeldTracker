#include "WTrackDType.h"
#include <cassert>

void test_enum_sizes() {
    static_assert(sizeof(WeldTrackApp::TcpCommStatus) == sizeof(int), "Enum size mismatch");
    static_assert(sizeof(WeldTrackApp::RobotRecvMsg) == (4 * 8 * 4 + 4 * 4), "Struct size mismatch");
}

void test_default_values() {
    WeldTrackApp::IncData incData;
    assert(incData.toolNo == WeldTrackApp::MacroDefine::toolNo);
    assert(incData.incDataType == 0);

    WeldTrackApp::RobotStatus status;
    assert(status.CurQueueCount == 0);
}

void test_array_access() {
    WeldTrackApp::RobotSendMsg sendMsg;
    sendMsg.outputIOvalue[0] = 1;
    assert(sendMsg.outputIOvalue[0] == 1);
}

int main() {
    test_enum_sizes();
    test_default_values();
    test_array_access();
    return 0;
}