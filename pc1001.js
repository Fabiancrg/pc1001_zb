/**
 * Zigbee2MQTT External Converter for PC1001 Pool Heat Pump Controller
 * 
 * This file defines the device for Zigbee2MQTT to properly expose all controls.
 * 
 * REPORTING FLAG OPTIMIZATION:
 * - Device uses ESP_ZB_ZCL_ATTR_ACCESS_REPORTING flag for automatic attribute reporting
 * - Temperature, setpoints, system_mode auto-report on changes
 * - Minimal polling only for attributes ESP-Zigbee stack cannot auto-report
 * 
 * ENDPOINTS:
 * - Endpoint 1: Main thermostat (heating/cooling control)
 * - Endpoint 2: Water outlet temperature sensor (TEMP_OUT)
 */

const fz = require('zigbee-herdsman-converters/converters/fromZigbee');
const tz = require('zigbee-herdsman-converters/converters/toZigbee');
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const reporting = require('zigbee-herdsman-converters/lib/reporting');
const m = require('zigbee-herdsman-converters/lib/modernExtend');
const e = exposes.presets;
const ea = exposes.access;

// Custom converters for thermostat
const fzLocal = {
    // Custom converter for thermostat attributes
    thermostat: {
        cluster: 'hvacThermostat',
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            const result = {};
            
            // Local temperature (water inlet)
            if (msg.data.hasOwnProperty('localTemp')) {
                result.local_temperature = msg.data.localTemp / 100;
            }
            
            // System mode
            if (msg.data.hasOwnProperty('systemMode')) {
                const sysModeMap = {
                    0x00: 'off',
                    0x01: 'auto',
                    0x03: 'cool',
                    0x04: 'heat',
                };
                result.system_mode = sysModeMap[msg.data.systemMode] || 'off';
            }
            
            // Temperature setpoint (use heating setpoint for both modes)
            if (msg.data.hasOwnProperty('occupiedHeatingSetpoint')) {
                result.occupied_heating_setpoint = msg.data.occupiedHeatingSetpoint / 100;
            } else if (msg.data.hasOwnProperty('occupiedCoolingSetpoint')) {
                result.occupied_heating_setpoint = msg.data.occupiedCoolingSetpoint / 100;
            }
            
            return result;
        },
    },
    
    // Custom converter for water outlet temperature (endpoint 2)
    water_outlet_temp: {
        cluster: 'msTemperatureMeasurement',
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            if (msg.endpoint.ID === 2 && msg.data.hasOwnProperty('measuredValue')) {
                const temp = msg.data.measuredValue;
                // Check for invalid temperature (0x8000 = invalid/unknown)
                if (temp === -32768 || temp === 0x8000) {
                    return {water_outlet_temperature: null};
                }
                return {water_outlet_temperature: temp / 100};
            }
        },
    },
};

const definition = {
    zigbeeModel: ['PC1001_ZB_v1'],
    model: 'PC1001-ZB',
    vendor: 'Custom devices (DiY)',
    description: 'PC1001 Pool Heat Pump Controller via Zigbee (End Device)',
    
    meta: {
        multiEndpoint: true,
    },
    
    // Supported features
    fromZigbee: [
        fzLocal.thermostat,         // Custom thermostat converter
        fzLocal.water_outlet_temp,  // Water outlet temperature sensor
    ],
    toZigbee: [
        tz.thermostat_local_temperature,
        tz.thermostat_occupied_heating_setpoint, // Single setpoint for both heating and cooling
        tz.thermostat_system_mode,
    ],
    
    // Expose controls in the UI
    exposes: [
        e.climate()
            .withSetpoint('occupied_heating_setpoint', 15, 32, 0.5)
            .withLocalTemperature()
            .withSystemMode(['off', 'auto', 'cool', 'heat'])
            .withDescription('Main thermostat - local_temperature is water inlet, setpoint controls target temperature'),
        exposes.numeric('water_outlet_temperature', exposes.access.STATE_GET)
            .withUnit('°C')
            .withDescription('Water outlet temperature from heat pump (read-only)'),
    ],
    
    // Map endpoints with descriptive names
    endpoint: (device) => {
        return {
            'default': 1,          // Main thermostat
            'water_outlet': 2,     // Water outlet temperature sensor
        };
    },
    
    // Configure reporting - with REPORTING flag, most attributes auto-report!
    configure: async (device, coordinatorEndpoint, logger) => {
        const endpoint1 = device.getEndpoint(1);
        const endpoint2 = device.getEndpoint(2);
        
        // Bind clusters for main thermostat (endpoint 1)
        await reporting.bind(endpoint1, coordinatorEndpoint, [
            'genBasic',
            'hvacThermostat',
        ]);
        
        // Configure reporting for thermostat attributes (with REPORTING flag, these auto-report!)
        await reporting.thermostatTemperature(endpoint1);  // localTemp (water inlet)
        await reporting.thermostatOccupiedHeatingSetpoint(endpoint1);  // setpoint
        
        // Configure systemMode reporting (0x001C)
        await endpoint1.configureReporting('hvacThermostat', [{
            attribute: 'systemMode',
            minimumReportInterval: 1,
            maximumReportInterval: 300,
            reportableChange: 1,
        }]);
        
        // Bind and configure water outlet temperature sensor (endpoint 2)
        await reporting.bind(endpoint2, coordinatorEndpoint, ['msTemperatureMeasurement']);
        await reporting.temperature(endpoint2, {min: 10, max: 300, change: 50});  // Report every 50 centidegrees (0.5°C)
        
        // Initial read of all attributes
        try {
            await endpoint1.read('hvacThermostat', ['localTemp', 'occupiedHeatingSetpoint', 'systemMode']);
            await endpoint2.read('msTemperatureMeasurement', ['measuredValue']);
        } catch (error) {
            logger.warn(`PC1001 configure: Initial read failed: ${error.message}`);
        }
    },
    
    // Minimal polling for status updates (most attributes now auto-report via REPORTING flag!)
    extend: [
        m.poll({
            key: "pc1001_state",
            option: e
                .numeric("pc1001_poll_interval", ea.SET)
                .withValueMin(-1)
                .withDescription(
                    "PC1001 polling interval for status updates. Default is 60 seconds. Most attributes auto-report via REPORTING flag! Set to -1 to disable."
                ),
            defaultIntervalSeconds: 60,
            poll: async (device) => {
                const endpoint1 = device.getEndpoint(1);
                const endpoint2 = device.getEndpoint(2);
                
                if (!endpoint1) {
                    console.warn(`PC1001 polling: endpoint 1 not found`);
                    return;
                }
                
                // Poll thermostat status
                try {
                    await endpoint1.read('hvacThermostat', ['localTemp', 'systemMode', 'occupiedHeatingSetpoint']);
                } catch (error) {
                    console.error(`PC1001 polling: thermostat read failed: ${error.message}`);
                }
                
                // Poll water outlet temperature
                if (endpoint2) {
                    try {
                        await endpoint2.read('msTemperatureMeasurement', ['measuredValue']);
                    } catch (error) {
                        console.error(`PC1001 polling: water outlet temp read failed: ${error.message}`);
                    }
                }
            },
        }),
    ],
    ota: true,
};

module.exports = definition;
