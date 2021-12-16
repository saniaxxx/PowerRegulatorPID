# -*- coding: utf-8 -*-
import os
import logging
import asyncio
import time
from cbpi.api import *

@parameters([Property.Actor(label="Heater")])
class TempSensorSimulator(CBPiSensor):
    def __init__(self, cbpi, id, props):
        super(TempSensorSimulator, self).__init__(cbpi, id, props)
        self.value = 0

    async def run(self):
        self._logger = logging.getLogger(type(self).__name__)
        while self.running:
            heater = self.get_actor(self.props.Heater)
            power = 0
            try:
                power = heater.instance.get_power()
                #self._logger.info("HEATER POWER IS %s" % power)
            except Exception as e:
                self._logger.error("CANNOT GET POWER OF ACTOR %s" % heater.id)
                self._logger.error(e)
            if power > 20:
                self.value = min(100, self.value + power * 0.01)
                #self._logger.info("VALUE INCREASES, %s" % self.value)
            elif power < 20:
                self.value = max(20, self.value - 0.1)
                #self._logger.info("VALUE DECREASES, %s" % self.value)
            self.log_data(self.value)
            self.push_update(self.value)
            await asyncio.sleep(1)

    def get_state(self):
        return dict(value=self.value)
      
@parameters([Property.Number(label = "P", configurable = True, default_value = 117.0795, description="P Value of PID"),
             Property.Number(label = "I", configurable = True, default_value = 0.2747, description="I Value of PID"),
             Property.Number(label = "D", configurable = True, default_value = 41.58, description="D Value of PID"),
             Property.Number(label = "Time", configurable = True, default_value = 5, description="Time of PID step"),
             Property.Number(label = "Max_Output", configurable = True, default_value = 100, description="Max power for PID and Ramp up."),
             Property.Number(label = "Boil_Threshold", configurable = True, description="When this temperature is reached, power will be set to Max Boil Output (default: 98 °C/208 F)"),
             Property.Number(label = "Max_Boil_Output", configurable = True, default_value = 85, description="Power when Boil Threshold is reached.")])
class PIDLogic(CBPiKettleLogic):
    async def run(self):
        try:
            loopTime = int(self.props.get("Time", 5))
            p = float(self.props.get("P", 70.0))
            i = float(self.props.get("I", 0.2))
            d = float(self.props.get("D", 20))
            m = float(self.props.get("Max_Output", 100))
            maxtempboil = float(self.props.get("Boil_Treshold", 98))
            maxboilout = int(self.props.get("Max_Boil_Output", 100))
            pid = PIDArduino(loopTime, p, i, d, 0, m)
            self.kettle = self.get_kettle(self.id)
            self.sensor = self.kettle.sensor
            self.heater = self.kettle.heater
            while self.running == True:
                if self.get_temp() >= float(maxtempboil):
                    heat_percent = maxboilout
                else:
                    heat_percent = pid.calc(self.get_temp(), self.get_target_temp())
                if heat_percent > 0:
                    await self.actor_on(self.heater, heat_percent)
                else:
                    await self.actor_off(self.heater)
                await asyncio.sleep(loopTime)
        except asyncio.CancelledError as e:
            pass
        except Exception as e:
            logging.error("PowerRegulatorPID Error {}".format(e))
        finally:
            self.running = False
            await self.actor_off(self.heater)

    async def actor_on(self, id, power):
        try:
            item = self.cbpi.actor.find_by_id(id)
            await item.instance.on(power=power)
            await self.cbpi.actor.push_udpate()
            self.cbpi.actor.cbpi.push_update("cbpi/actor/"+id, item.to_dict(), True)
        except Exception as e:
            logging.error("PowerRegulatorPID Error {}".format(e))

    def get_temp(self):
        return float(self.get_sensor_value(self.sensor).get("value"))

    def get_target_temp(self):
        return int(self.get_kettle_target_temp(self.id))

# Based on Arduino PID Library
# See https://github.com/br3ttb/Arduino-PID-Library
class PIDArduino(object):

    def __init__(self, sampleTimeSec, kp, ki, kd, outputMin=float('-inf'),
                 outputMax=float('inf'), getTimeMs=None):
        if kp is None:
            raise ValueError('kp must be specified')
        if ki is None:
            raise ValueError('ki must be specified')
        if kd is None:
            raise ValueError('kd must be specified')
        if sampleTimeSec <= 0:
            raise ValueError('sampleTimeSec must be greater than 0')
        if outputMin >= outputMax:
            raise ValueError('outputMin must be less than outputMax')

        self._logger = logging.getLogger(type(self).__name__)
        self._Kp = kp
        self._Ki = ki * sampleTimeSec
        self._Kd = kd / sampleTimeSec
        self._sampleTime = sampleTimeSec * 1000
        self._outputMin = outputMin
        self._outputMax = outputMax
        self._iTerm = 0
        self._lastInput = 0
        self._lastOutput = 0
        self._lastCalc = 0

        if getTimeMs is None:
            self._getTimeMs = self._currentTimeMs
        else:
            self._getTimeMs = getTimeMs

    def calc(self, inputValue, setpoint):
        now = self._getTimeMs()

        if (now - self._lastCalc) < self._sampleTime:
            return self._lastOutput

        # Compute all the working error variables
        error = setpoint - inputValue
        dInput = inputValue - self._lastInput

        # In order to prevent windup, only integrate if the process is not saturated
        if self._lastOutput < self._outputMax and self._lastOutput > self._outputMin:
            self._iTerm += self._Ki * error
            self._iTerm = min(self._iTerm, self._outputMax)
            self._iTerm = max(self._iTerm, self._outputMin)

        p = self._Kp * error
        i = self._iTerm
        d = -(self._Kd * dInput)
        
        # Compute PID Output
        self._lastOutput = p + i + d
        self._lastOutput = min(self._lastOutput, self._outputMax)
        self._lastOutput = max(self._lastOutput, self._outputMin)

        # Log some debug info
        self._logger.debug('P: {0}'.format(p))
        self._logger.debug('I: {0}'.format(i))
        self._logger.debug('D: {0}'.format(d))
        self._logger.debug('output: {0}'.format(self._lastOutput))

        # Remember some variables for next time
        self._lastInput = inputValue
        self._lastCalc = now
        return self._lastOutput

    def _currentTimeMs(self):
        return time.time() * 1000

def setup(cbpi):
    cbpi.plugin.register("PIDLogic", PIDLogic)
    cbpi.plugin.register("TempSensorSimulator", TempSensorSimulator)
