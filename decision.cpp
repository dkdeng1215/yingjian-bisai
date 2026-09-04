#include "decision.h"
#include "config.h"

namespace {
const uint16_t HYSTERESIS_MM = 50;
const uint16_t WIDE_DIFF_MM = 120;

bool obstacle(const ZoneState &z) {
  return z.valid && classifyRisk(z.distanceMm) != SAFE;
}
}

RiskLevel classifyRisk(uint16_t distanceMm) {
  if (distanceMm < DANGER_THRESHOLD_MM) return DANGER;
  if (distanceMm < WARN_THRESHOLD_MM) return WARN;
  if (distanceMm < NOTICE_THRESHOLD_MM) return NOTICE;
  return SAFE;
}

RiskLevel classifyRiskWithHysteresis(RiskLevel previousRisk,
                                     uint16_t distanceMm) {
  RiskLevel instant = classifyRisk(distanceMm);
  // 靠近时立即升级；传感器重置/非法枚举时从即时档开始。
  if (previousRisk < SAFE || previousRisk > DANGER) return instant;
  if (instant > previousRisk) return instant;

  // 远离时每次最多降一档，且必须越过当前档位阈值 + 50mm。
  RiskLevel result = previousRisk;
  if (result == DANGER && distanceMm >= DANGER_THRESHOLD_MM + HYSTERESIS_MM)
    result = WARN;
  if (result == WARN && distanceMm >= WARN_THRESHOLD_MM + HYSTERESIS_MM)
    result = NOTICE;
  if (result == NOTICE && distanceMm >= NOTICE_THRESHOLD_MM + HYSTERESIS_MM)
    result = SAFE;
  return result;
}

SystemState buildSystemState(const ZoneState zones[3]) {
  SystemState state;
  state.anySensorInvalid = false;
  state.allSensorsInvalid = true;
  for (uint8_t i = 0; i < 3; ++i) {
    state.zones[i] = zones[i];
    if (zones[i].valid) {
      state.allSensorsInvalid = false;
    } else {
      state.anySensorInvalid = true;
    }
  }
  return state;
}

bool isNearestObstacle(Direction candidate, const ZoneState zones[3]) {
  if (candidate > RIGHT || !obstacle(zones[candidate])) return false;
  const uint16_t d = zones[candidate].distanceMm;
  for (uint8_t i = 0; i < 3; ++i) {
    if (i != static_cast<uint8_t>(candidate) && obstacle(zones[i]) &&
        zones[i].distanceMm < d)
      return false;
  }
  return true;
}

bool isNearestObstacle(const ZoneState &candidate, const ZoneState zones[3]) {
  if (!obstacle(candidate)) return false;
  for (uint8_t i = 0; i < 3; ++i)
    if (obstacle(zones[i]) && zones[i].distanceMm < candidate.distanceMm)
      return false;
  return true;
}

bool isWideObstacle(const ZoneState &left, const ZoneState &right) {
  if (!obstacle(left) || !obstacle(right)) return false;
  uint16_t diff = left.distanceMm > right.distanceMm
                      ? left.distanceMm - right.distanceMm
                      : right.distanceMm - left.distanceMm;
  return diff <= WIDE_DIFF_MM;
}

FeedbackDecision decideFeedback(const SystemState &state) {
  static RiskLevel lastRisk[3] = {SAFE, SAFE, SAFE};
  FeedbackDecision decision = {0, SAFE, false};
  if (state.allSensorsInvalid) {
    lastRisk[LEFT] = lastRisk[CENTER] = lastRisk[RIGHT] = SAFE;
    return decision;
  }
  int nearest = -1;
  uint16_t nearestDistance = 0xffff;
  for (uint8_t i = 0; i < 3; ++i) {
    if (!state.zones[i].valid) { lastRisk[i] = SAFE; continue; }
    lastRisk[i] = classifyRiskWithHysteresis(lastRisk[i], state.zones[i].distanceMm);
    if (lastRisk[i] != SAFE && state.zones[i].distanceMm < nearestDistance) {
      nearest = i; nearestDistance = state.zones[i].distanceMm;
    }
  }
  if (nearest < 0) return decision;
  decision.activeDirections = static_cast<uint8_t>(1u << nearest);
  const bool wide = state.zones[LEFT].valid && state.zones[RIGHT].valid &&
                    lastRisk[LEFT] != SAFE && lastRisk[RIGHT] != SAFE &&
                    isWideObstacle(state.zones[LEFT], state.zones[RIGHT]);
  if (wide) {
    decision.activeDirections = (1u << LEFT) | (1u << RIGHT);
    decision.isWideObstacle = true;
    if (lastRisk[CENTER] != SAFE &&
        state.zones[CENTER].distanceMm < state.zones[LEFT].distanceMm &&
        state.zones[CENTER].distanceMm < state.zones[RIGHT].distanceMm)
      decision.activeDirections |= (1u << CENTER);
  }
  // risk 取所有激活方向中最高的风险等级
  RiskLevel maxRisk = SAFE;
  for (uint8_t i = 0; i < 3; ++i) {
    if (decision.activeDirections & (1u << i)) {
      if (lastRisk[i] > maxRisk) {
        maxRisk = lastRisk[i];
      }
    }
  }
  decision.risk = maxRisk;
  return decision;
}

const char *riskName(RiskLevel risk) {
  switch (risk) {
    case SAFE: return "SAFE";
    case NOTICE: return "NOTICE";
    case WARN: return "WARN";
    case DANGER: return "DANGER";
    default: return "UNKNOWN";
  }
}

bool isActiveRisk(RiskLevel risk) { return risk != SAFE; }
