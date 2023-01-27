from pathlib import Path

from cereal import car
from common.numpy_fast import clip
from opendbc.can.packer import CANPacker
from selfdrive.car import apply_std_steer_angle_limits
from selfdrive.car.ford.fordcan import create_acc_ui_msg, create_button_msg, create_lat_ctl_msg, create_lka_msg, create_lkas_ui_msg
from selfdrive.car.ford.values import CANBUS, CarControllerParams

VisualAlert = car.CarControl.HUDControl.VisualAlert


class CarController:
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.VM = VM
    self.packer = CANPacker(dbc_name)
    self.frame = 0

    self.apply_curvature_last = 0
    self.main_on_last = False
    self.lkas_enabled_last = False
    self.steer_alert_last = False

  def update(self, CC, CS):
    can_sends = []

    actuators = CC.actuators
    hud_control = CC.hudControl

    main_on = CS.out.cruiseState.available
    steer_alert = hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw)

    ### acc buttons ###
    if CC.cruiseControl.cancel:
      can_sends.append(create_button_msg(self.packer, CS.buttons_stock_values, cancel=True))
      can_sends.append(create_button_msg(self.packer, CS.buttons_stock_values, cancel=True, bus=CANBUS.main))
    elif CC.cruiseControl.resume and (self.frame % CarControllerParams.BUTTONS_STEP) == 0:
      can_sends.append(create_button_msg(self.packer, CS.buttons_stock_values, resume=True))
      can_sends.append(create_button_msg(self.packer, CS.buttons_stock_values, resume=True, bus=CANBUS.main))
    # if stock lane centering isn't off, send a button press to toggle it off
    # the stock system checks for steering pressed, and eventually disengages cruise control
    elif CS.acc_tja_status_stock_values["Tja_D_Stat"] != 0 and (self.frame % CarControllerParams.ACC_UI_STEP) == 0:
      can_sends.append(create_button_msg(self.packer, CS.buttons_stock_values, tja_toggle=True))

    exp_1 = Path("/data/Exp1").is_file()
    exp_2 = Path("/data/Exp2").is_file()
    exp_3 = Path("/data/Exp3").is_file()
    exp_4 = Path("/data/Exp4").is_file()
    exp_5 = Path("/data/Exp5").is_file()
    exp_6 = Path("/data/Exp6").is_file()
    exp_7 = Path("/data/Exp7").is_file()

    ### lateral control ###
    # send steering commands at 20Hz
    if (self.frame % CarControllerParams.STEER_STEP) == 0:
      if CC.latActive:
        # apply limits to curvature and clip to signal range
        apply_curvature = apply_std_steer_angle_limits(actuators.curvature, self.apply_curvature_last, CS.out.vEgo, CarControllerParams)
        apply_curvature = clip(apply_curvature, -CarControllerParams.CURVATURE_MAX, CarControllerParams.CURVATURE_MAX)

        exp_mode = 7 if exp_7 else 6 if exp_6 else 5 if exp_5 else 4 if exp_4 else 3 if exp_3 else 2 if exp_2 else 1 if exp_1 else 0
        mode = exp_mode if CS.out.steeringPressed else 1
      else:
        apply_curvature = 0.
        mode = 0

      self.apply_curvature_last = apply_curvature
      can_sends.append(create_lka_msg(self.packer))
      can_sends.append(create_lat_ctl_msg(self.packer, mode, 0., 0., -apply_curvature, 0.))

    ### ui ###
    send_ui = (self.main_on_last != main_on) or (self.lkas_enabled_last != CC.latActive) or (self.steer_alert_last != steer_alert)

    # send lkas ui command at 1Hz or if ui state changes
    if (self.frame % CarControllerParams.LKAS_UI_STEP) == 0 or send_ui:
      can_sends.append(create_lkas_ui_msg(self.packer, main_on, CC.latActive, steer_alert, hud_control, CS.lkas_status_stock_values))

    # send acc ui command at 20Hz or if ui state changes
    if (self.frame % CarControllerParams.ACC_UI_STEP) == 0 or send_ui:
      can_sends.append(create_acc_ui_msg(self.packer, main_on, CC.latActive, hud_control, CS.acc_tja_status_stock_values))

    self.main_on_last = main_on
    self.lkas_enabled_last = CC.latActive
    self.steer_alert_last = steer_alert

    new_actuators = actuators.copy()
    new_actuators.curvature = self.apply_curvature_last

    self.frame += 1
    return new_actuators, can_sends
