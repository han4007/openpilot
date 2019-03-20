#!/usr/bin/env python
import os
import zmq
import time
from selfdrive.can.parser import CANParser
from cereal import car
from common.realtime import sec_since_boot
from selfdrive.services import service_list
import selfdrive.messaging as messaging
from selfdrive.car.toyota.values import NO_DSU_CAR


RADAR_C_MSGS = list(range(0x680, 0x685))

def _create_radard_can_parser():
  dbc_f = 'Toyota_Conti_Radar.dbc'

  msg_c_n = len(RADAR_C_MSGS)

  signals = zip(['LONG_DIST'] * msg_c_n + ['LAT_DIST'] * msg_c_n +
                ['REL_SPEED'] * msg_c_n + ['VALID'] * msg_c_n + ['SCORE'] * msg_c_n,
                RADAR_C_MSGS * 5,
                [255] * msg_c_n + [1] * msg_c_n + [0] * msg_c_n + [0] * msg_c_n + [0] * msg_c_n)

  checks = zip(RADAR_C_MSGS, [20])

  return CANParser(os.path.splitext(dbc_f)[0], signals, checks, 1)


class RadarInterface(object):
  def __init__(self, CP):
    # radar
    self.pts = {}
    self.valid_cnt = {key: 0 for key in RADAR_A_MSGS}
    self.track_id = 0

    self.delay = 0.0  # Delay of radar

    self.rcp = _create_radard_can_parser()

    context = zmq.Context()
    self.logcan = messaging.sub_sock(context, service_list['can'].port)

  def update(self):

    ret = car.RadarState.new_message()
    canMonoTimes = []
    updated_messages = set()
    while 1:
      tm = int(sec_since_boot() * 1e9)
      updated_messages.update(self.rcp.update(tm, True))
      if RADAR_B_MSGS[-1] in updated_messages:
        break

    errors = []
    if not self.rcp.can_valid:
      errors.append("commIssue")
    ret.errors = errors
    ret.canMonoTimes = canMonoTimes

    for ii in updated_messages:
      if ii in RADAR_A_MSGS:
        cpt = self.rcp.vl[ii]

        if cpt['LONG_DIST'] >=255 or cpt['NEW_TRACK']:
          self.valid_cnt[ii] = 0    # reset counter
        if cpt['VALID'] and cpt['LONG_DIST'] < 255:
          self.valid_cnt[ii] += 1
        else:
          self.valid_cnt[ii] = max(self.valid_cnt[ii] -1, 0)

        score = self.rcp.vl[ii+16]['SCORE']
        # print ii, self.valid_cnt[ii], score, cpt['VALID'], cpt['LONG_DIST'], cpt['LAT_DIST']

        # radar point only valid if it's a valid measurement and score is above 50
        if cpt['VALID'] or (score > 50 and cpt['LONG_DIST'] < 255 and self.valid_cnt[ii] > 0):
          if ii not in self.pts or cpt['NEW_TRACK']:
            self.pts[ii] = car.RadarState.RadarPoint.new_message()
            self.pts[ii].trackId = self.track_id
            self.track_id += 1
          self.pts[ii].dRel = cpt['LONG_DIST']  # from front of car
          self.pts[ii].yRel = -cpt['LAT_DIST']  # in car frame's y axis, left is positive
          self.pts[ii].vRel = cpt['REL_SPEED']
          self.pts[ii].aRel = float('nan')
          self.pts[ii].yvRel = float('nan')
          self.pts[ii].measured = bool(cpt['VALID'])
        else:
          if ii in self.pts:
            del self.pts[ii]

    ret.points = self.pts.values()
    return ret

if __name__ == "__main__":
  RI = RadarInterface(None)
  while 1:
    ret = RI.update()
    print(chr(27) + "[2J")
    print(ret)
