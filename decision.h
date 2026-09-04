#ifndef DECISION_H
#define DECISION_H

#include "common.h"

// 即时风险分档（不带滞回）：距离严格小于阈值才进入更高风险。
RiskLevel classifyRisk(uint16_t distanceMm);

// 根据上一档风险和当前距离计算带 50mm 滞回的风险。
RiskLevel classifyRiskWithHysteresis(RiskLevel previousRisk,
                                     uint16_t distanceMm);

// 汇总三路状态并计算传感器异常标志。
SystemState buildSystemState(const ZoneState zones[3]);

// candidate 是否为有效、非 SAFE 且距离最近的障碍方向。
bool isNearestObstacle(Direction candidate, const ZoneState zones[3]);
// 兼容按状态传入的调用方式；同距离时视为并列最近。
bool isNearestObstacle(const ZoneState &candidate, const ZoneState zones[3]);

// 左右均为有效障碍且距离差不超过 120mm。
bool isWideObstacle(const ZoneState &left, const ZoneState &right);

// 根据系统状态生成反馈决策。
FeedbackDecision decideFeedback(const SystemState &state);

// 日志/反馈辅助函数。
const char *riskName(RiskLevel risk);
bool isActiveRisk(RiskLevel risk);

#endif
