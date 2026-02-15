	var isTest = (location.hostname === "localhost" || location.hostname === "127.0.0.1" || location.href.indexOf("file://") != -1);

var _ucAuthToken = '';
var _ucAuthPromptActive = false;
var _ucUserInteracted = false;
try {
    _ucAuthToken = sessionStorage.getItem('ucAuthToken') || '';
} catch(e) {
    _ucAuthToken = '';
}

function setUcAuthToken(token){
    var prev = _ucAuthToken;
    _ucAuthToken = token || '';
    try {
        if(_ucAuthToken){
            sessionStorage.setItem('ucAuthToken', _ucAuthToken);
        } else {
            sessionStorage.removeItem('ucAuthToken');
        }
    } catch(e) {}
    if(prev !== _ucAuthToken){
        applyMenuAccessMode();
    }
}

function isDashboardOnlyMode(){
    return (!isTest && !_ucAuthToken);
}

function updateMenuLockBadge(){
    var locked = isDashboardOnlyMode();
    var $badge = $('#menuLockBadge');
    if(!$badge.length) return;
    if(locked){
        $badge.removeClass('hidden');
    } else {
        $badge.addClass('hidden');
    }
}

function applyMenuAccessMode(){
    if(!$('#menu tr').length) return;
    setLocalPagination();
    updateMenuLockBadge();
    bindMenuHandlers();
    var $active = $('.menu-item.active');
    if($active.length && !isDashboardOnlyMode()){
        $active.trigger('click');
    } else {
        $('.menu-item[pagemenu="main"]').trigger('click');
    }
}

function bindMenuHandlers(){
    $('[pagemenu]').off('click.ucmenu').on('click.ucmenu',function(){
        if(isDashboardOnlyMode() && $(this).attr('pagemenu') !== 'main'){
            return;
        }
        $('.menu-item.active').removeClass('active');
        $('[page].active').removeClass('active');
        $(this).addClass('active');
        var targetPage = $(this).attr('pagemenu');
        $('[page="'+targetPage+'"]').removeAttr('hidden');
        $('[page="'+targetPage+'"]').addClass('active');
        if(targetPage == 'service'){
            getClockDetails();
        }
        if(targetPage == 'system'){
            fetchSystemInfo();
        }
    });

    $('#menuLockBadge').off('click.ucmenu').on('click.ucmenu', function(){
        if(!isDashboardOnlyMode()) return;
        promptAuthLogin();
    });
}

$.ajaxPrefilter(function(options, originalOptions, jqXHR){
    if(!_ucAuthToken) return;
    var url = String(options && options.url ? options.url : '');
    if(url.indexOf('/auth/login') !== -1) return;
    jqXHR.setRequestHeader('X-Auth-Token', _ucAuthToken);
});

function apiGet(url, data){
    return $.ajax({ url: url, method: 'GET', data: data || {} });
}

function apiPost(url, data){
    var payload = data || {};
    var d = $.Deferred();

    $.ajax({ url: url, method: 'POST', data: payload })
        .done(function(resp){
            d.resolve(resp);
        })
        .fail(function(jqxhr){
            if(!jqxhr || jqxhr.status !== 401){
                d.reject(jqxhr);
                return;
            }
            setUcAuthToken('');
            if(!_ucUserInteracted){
                d.reject(jqxhr);
                return;
            }
            promptAuthLogin().done(function(){
                $.ajax({ url: url, method: 'POST', data: payload })
                    .done(function(resp2){ d.resolve(resp2); })
                    .fail(function(j2){ d.reject(j2); });
            }).fail(function(){
                d.reject(jqxhr);
            });
        });

    return d.promise();
}

function promptAuthLogin(){
    if(isTest){
        var d0 = $.Deferred();
        d0.resolve();
        return d0.promise();
    }
    if(_ucAuthPromptActive){
        var dBusy = $.Deferred();
        dBusy.reject('auth_in_progress');
        return dBusy.promise();
    }
    _ucAuthPromptActive = true;

    var password = window.prompt('Enter ClockForgeOS admin password');
    if(!password){
        _ucAuthPromptActive = false;
        var d1 = $.Deferred();
        d1.reject('auth_cancelled');
        return d1.promise();
    }

    return $.post('/auth/login', { password: password }).done(function(resp){
        if (typeof resp === 'string') {
            try { resp = JSON.parse(resp); } catch(e){ resp = {}; }
        }
        if(resp && resp.token){
            setUcAuthToken(resp.token);
            getConfiguration();
        }
    }).always(function(){
        _ucAuthPromptActive = false;
    });
}

function ensureAuth(){
    if(isTest || _ucAuthToken){
        var d2 = $.Deferred();
        d2.resolve();
        return d2.promise();
    }
    return promptAuthLogin();
}

$(document).ajaxError(function(event, jqxhr, settings){
    if(!jqxhr || jqxhr.status !== 401) return;
    var url = String((settings && settings.url) ? settings.url : '');
    if(url.indexOf('/auth/login') !== -1) return;
    setUcAuthToken('');
});

var controlInfos = {
    "version": '',
    "maxDigits": "Number of tubes",
    "maxBrightness": "MAX Brightness",
    "currentDate": "Date",
    "currentTime": "Time",
    "temperature": "Temperature sensor #1",
	"temperature2": "Temperature sensor #2",
    "humidity": "Humidity sensor #1",
	"humidity2": "Humidity sensor #2",
	"pressure": "Pressure sensor #1",
	"lux": "Lux sensor current value",
	"rssi": "WiFi RSSI level dB (connection quality)",
    "wifiMode": "WiFi Client or Access Point",
	"alarmEnable": "Switch ON/OFF alarm",
    "alarmTime": "Alarm time (hour/minute)",
	"alarmPeriod": "Alarm maximum length (sec)",
	
	//Tube display settings
    "utc_offset": 'Timezone setting. <a href="https://en.wikipedia.org/wiki/List_of_time_zones_by_country" target="_blank"></a>https://en.wikipedia.org/wiki/List_of_time_zones_by_country',
    "enableDST": "Auto Daylight Saving Time ON/OFF",
    "set12_24": "Clock display 12 / 24 hours",
    "showZero": "Show starting zero on hours",
    "enableBlink": "ON / OFF",
    "interval": "Cathode protect interval (minutes)",
    "enableAutoShutoff": "Enable automatic day/night switch",
    "dayTime": "Day starts (manual brightness control)",
    "nightTime": "Night starts (manual brightness control)",
    "dayBright": "Day brightness",
    "nightBright": "Night brightness",
	"manualOverride": "Manual Day/Night change",
    "animMode": "Tube animation mode: 0=OFF, 1-5: Animations, 7: Random(1-4,8,9), 8: Slot Wave, 9: Mid to Margin",
	"dateMode": "0:dd/mm/yyyy 1:mm/dd/yyyy 2:yyyy/mm/dd",
	"tempCF": "Celsius / Fahrenheit display",              
	"enableTimeDisplay": "Enable time&date display (Use OFF for thermometers)" ,
    "enableTempDisplay": "Enable/disable temperature display on tubes",
    "enableHumidDisplay": "Enable/disable humidity display on tubes",
    "enablePressDisplay": "Enable/disable pressure display on tubes",
	"dateStart": "Date display start: Date is shown between Start/End seconds",
	"dateEnd": "Date display end",    
	"tempStart": "Temperature display time slice start",
	"tempEnd": "Temperature display end",  
	"humidStart": "Humidity display time slice start",
	"humidEnd": "Humidity display end",
    "pressureStart": "Pressure display time slice start",
    "pressureEnd": "Pressure display end",
	"dateRepeatMin": "Date is shown in every xx minutes. 0 = Disable date display",     
    "tempRepeatMin": "Temperature/humidity/pressure is shown in every xx minutes. 0 = Disable temp/humid/pressure display",  	
	"enableDoubleBlink": "Enable both blinking dots",             
	"enableAutoDim": "Enable auto brightness control (if sensor is installed)",  
	"enableRadar": "Enable Radar/Pir tube switch (if sensor is installed)",   
	"radarTimeout": "If Radar is used, automatic switch off after this timeout (minutes)",         

    //RGB settings	
    "rgbDir" : "RGB animation direction (left/right)",
    "rgbEffect": "RGB animation#",
    "rgbBrightness": "RGB leds brightness 0..255",    "rgbFixR": "Fixed RGB red (0..255) used when set",
    "rgbFixG": "Fixed RGB green (0..255) used when set",
    "rgbFixB": "Fixed RGB blue (0..255) used when set",
    "maxLedmA": "Maximum NeoPixel current budget in mA (0 disables limiter)",
    "rgbSpeed": "RGB animation speed",
	
    //Wifi settings
	"wifiSsid": "WiFi network SSID to connect",
	"wifiPsw": "WiFI password",
	"ApSsid": "AccessPoint name, if standalone mode is used",
	"ApPsw": "AP password",
	"NtpServer": "Timeserver name, using WiFi. (Default is: pool.ntp.org)",
	"mqttBrokerAddr": "MQTT server address, if MQTT used", 
	"mqttBrokerUser": "MQTT user",
	"mqttBrokerPsw": "MQTT password",
	"mqttBrokerRefresh": "MQTT send data in every xx sec",
	"mqttEnable": "Enable/Disable MQTT refresh",
	"firmware": "http server + firmware name to download new firmware",
	"corrT0": "Temperature sensor#1 correction",
	"corrT1": "Temperature sensor#2 correction",
	"corrH0": "Humidity sensor#1 correction",
	"corrH1": "Humidity sensor#2 correction"	,
	"cathProtMin": "Cathode Protect procedure (minutes)",
    "onboardLed": "ESP32 built-in LED manual ON/OFF",
    "touchShortAction": "GPIO33 touch short press action",
    "touchDoubleAction": "GPIO33 touch double press action",
    "touchLongAction": "GPIO33 touch long press action",
    "gestureUpAction": "Gesture UP action mapping",
    "gestureDownAction": "Gesture DOWN action mapping",
    "gestureLeftAction": "Gesture LEFT action mapping",
    "gestureRightAction": "Gesture RIGHT action mapping",
	"FW": "firmware code",
	"tubeDriver": "Tube driver modul"
};

//Example config that UI recieves
var configuration = {
	//Main screen	
    "version": "ClockForgeOS 1.0",
    "maxDigits": 6,
    "maxBrightness": 10,
    "currentDate": "2024.11.6",
    "currentTime": "14:10",	
    "temperature": 22.1,
	"temperature2": 255,
    "humidity": 45,
	"humidity2": 255,
	"pressure": 255,
	"lux": 100,
	"rssi": 0,
    "telnetClients": 0,
    "mqttClients": 0,
    "mqttStatus": "Disconnected",
	"wifiMode": true,
    "alarmEnable": false,
    "alarmTime": "6:30",
	"alarmPeriod": 15,
	"enableRadar": false,   
	"radarTimeout": 5,    	
	
	//Tube display settings
    "utc_offset": 1,
    "enableDST": true,
    "set12_24": true,
    "showZero": true,
    "enableBlink": true,
    "interval": 15,
    "enableAutoShutoff": true,
    "dayTime": "7:00",
    "nightTime": "22:00",
    "dayBright": 10,
    "nightBright": 3,
	"manualOverride": true,
    "animMode": 6,
	"dateMode": 2,              // 0:dd/mm/yyyy 1:mm/dd/yyyy 2:yyyy/mm/dd
	"tempCF": false,               //Temperature Celsius / Fahrenheit
	"enableTimeDisplay": true,       
    "enableTempDisplay": true,
    "enableHumidDisplay": true,
    "enablePressDisplay": true,
    "dateStart": 35,
    "dateEnd": 40,
    "tempStart": 40,
    "tempEnd": 45,
    "humidStart": 45,
    "humidEnd": 50,
    "pressureStart": 50,
    "pressureEnd": 55,
	"dateRepeatMin": 3, 
	"tempRepeatMin": 1,            	
	"enableDoubleBlink": true,             
	"enableAutoDim": false,  
 
    //RGB settings	
    "rgbDir" : true,
    "rgbEffect": 1,
    "rgbBrightness": 100,    "rgbFixR": 255,
    "rgbFixG": 255,
    "rgbFixB": 255,
    "maxLedmA": 350,
    "rgbSpeed": 50,
	
    //Wifi settings
	"wifiSsid": "mywifi",
	"wifiPsw": "mypsw",
    "ApSsid": "ClockForgeOS",
    "ApPsw": "clockforgeos",
	"NtpServer": "pool.ntp.org",
	"mqttBrokerAddr": "10.0.99.12", 
	"mqttBrokerUser": "mqtt",
	"mqttBrokerPsw": "mqttPW",
	"mqttBrokerRefresh": 30,
	"mqttEnable": false,
    "extDev0Name": "",
    "extDev0Topic": "",
    "extDev0Slot": 0,
    "extDev1Name": "",
    "extDev1Topic": "",
    "extDev1Slot": 1,
    "extDev2Name": "",
    "extDev2Topic": "",
    "extDev2Slot": 2,
    "extDev3Name": "",
    "extDev3Topic": "",
    "extDev3Slot": 3,
	"firmware": "",
	"corrT0": 0.0,
	"corrT1": 0.0,
	"corrH0": 0.0,
	"corrH1": 0.0,
	"cathProtMin": 5,
    "onboardLed": false,
    "touchShortAction": 0,
    "touchDoubleAction": 0,
    "touchLongAction": 0,
    "gestureUpAction": 2,
    "gestureDownAction": 5,
    "gestureLeftAction": 5,
    "gestureRightAction": 2,
    "gestureSensorPresent": false,
    "FW": "fw",
	"tubeDriver": "xxx"
};

var isMouseDown = 0;
// ---- RGB EEPROM debounce ----
var _rgbCommitTimer = null;
var _maxLedCommitTimer = null;
function commitRgbDebounced(r,g,b){
    clearTimeout(_rgbCommitTimer);
    _rgbCommitTimer = setTimeout(function(){
        sendMsgToArduino('rgbFixR', r);
        sendMsgToArduino('rgbFixG', g);
        sendMsgToArduino('rgbFixB', b);
    }, 900); // 0.9 sec delay
}
//Runs, when HTML document is fully loaded
$(document).ready(function(){    
    $(document).on('click keydown touchstart change', function(){
        _ucUserInteracted = true;
    });
    $(document).on('mousedown touchstart',function(event) {
        ++isMouseDown;
    });
    $(document).on('mouseup touchend',function(event) {
        --isMouseDown;
    });

    if(isTest){
        Init();
        setCurrentInfos();
    }
    else{
        getConfiguration(); 
        //Set current infos, like time, humidity and temperature
        getCurrentInfos();
    }

    initMainExtrasBindings();
    initLiveLog();
    initFeatureModules();
});

function setLocalPagination(){
    var menuLabels = {
        main: 'Dashboard',
        display: 'Display',
        automation: 'Automation',
        settings: 'Settings',
        service: 'Service',
        rgb: 'RGB',
        wifi: 'Network',
        ui: 'UI',
        system: 'System'
    };

    var dashboardOnly = isDashboardOnlyMode();
    $('#menu tr').empty();
    $('[page]').not('[hidden]').each(function(){
        var page = $(this).attr('page');
        if(dashboardOnly && page !== 'main') return;
        var label = menuLabels[page] || page;
        $('#menu tr').append('<td><div class="menu-item" pagemenu="'+page+'">'+label+'</div></td>');
    });
}

function getConfiguration(){
    var cfgUrl = (!isTest && !_ucAuthToken) ? '/getPublicConfig' : '/getConfiguration';
    apiGet(cfgUrl).done(function(data){
        configuration = $.extend({}, configuration, data || {});
        applyConfigToMainExtras(configuration);
        // Apply UI customization settings from firmware EEPROM
        loadUICSSFromLocalStorage();
    }).always(function(){
        Init();
    });
}

function sendMsgToArduino(key,value) {
    if(isTest){
        console.log(key + " " + value);
    }
    else{
        // Always send as string to keep backend parsing predictable (form posts arrive as strings).
        apiPost('/saveSetting', {"key" : key ,"value" : String(value) }).done(function(data){
            console.log(data);
        });
    }
}

var COLORSATURATION = 255;
var WHITE_INDEX = 192;
function getColorFrom(WheelPos){
    if (WheelPos == WHITE_INDEX) 
        return RgbColor(COLORSATURATION/2, COLORSATURATION/2, COLORSATURATION/2);
        
    WheelPos = 255 - WheelPos;
    if(WheelPos < 85)  {
        return RgbColor(255 - WheelPos * 3, 0, WheelPos * 3);
    } else if(WheelPos < 170) {
        WheelPos -= 85;
        return RgbColor(0, WheelPos * 3, 255 - WheelPos * 3);
    } else  {
        WheelPos -= 170;
        return RgbColor(WheelPos * 3, 255 - WheelPos * 3, 0);
    }
}

function RgbColor(r, g, b){
    return 'rgb('+r+','+g+','+b+')';
}

function hexToRgb(hex){
    if(!hex) return null;
    hex = String(hex).trim();
    if(hex[0] === '#') hex = hex.slice(1);
    if(hex.length === 3){
        hex = hex.split('').map(function(c){ return c + c; }).join('');
    }
    if(hex.length !== 6) return null;
    var num = parseInt(hex, 16);
    if(Number.isNaN(num)) return null;
    return { r:(num>>16)&255, g:(num>>8)&255, b:num&255 };
}

function rgbToHex(r,g,b){
    function to2(v){ v = Math.max(0, Math.min(255, parseInt(v,10)||0)); return v.toString(16).padStart(2,'0'); }
    return "#" + to2(r) + to2(g) + to2(b);
}

function setPreviewColor(){
    // Prefer custom RGB picker if present and set
    var hasPicker = $('#rgbFixPicker').length > 0;
    var r = configuration["rgbFixR"], g = configuration["rgbFixG"], b = configuration["rgbFixB"];
    if(hasPicker && r !== undefined && g !== undefined && b !== undefined){
        $('#rgbFixPicker').val(rgbToHex(r,g,b));
        if($('#rgbFixR').length) $('#rgbFixR').val(r);
        if($('#rgbFixG').length) $('#rgbFixG').val(g);
        if($('#rgbFixB').length) $('#rgbFixB').val(b);
        if($('#rgbFixRval').length) $('#rgbFixRval').text(r);
        if($('#rgbFixGval').length) $('#rgbFixGval').text(g);
        if($('#rgbFixBval').length) $('#rgbFixBval').text(b);
        return;
    }
}

function setCurrentInfos(){
    var currentDateText = getCurrentDate(configuration["currentDate"]);
    var currentTimeText = String(configuration["currentTime"] || '--:--');
    $('#currentTime').html('<span class="current-time-nixie">' + currentTimeText + '</span><span class="current-time-date">' + currentDateText + '</span>');
    $('#lux').html(configuration["lux"]);
	$('#rssi').html(configuration["rssi"]);
    $('#telnetClients').html((configuration["telnetClients"] !== undefined) ? configuration["telnetClients"] : 0);
    $('#mqttClients').html((configuration["mqttClients"] !== undefined) ? configuration["mqttClients"] : 0);
    var mqttConnected = (configuration["mqttStatus"] === "Connected") || (parseInt(configuration["mqttClients"] || 0, 10) > 0);
    $('#mqttMainStatus')
        .text(mqttConnected ? 'Active' : 'Inactive')
        .toggleClass('mqtt-state-on', mqttConnected)
        .toggleClass('mqtt-state-off', !mqttConnected);
    var pressureVal = parseFloat(configuration["pressure"]);
    var pressureValid = !Number.isNaN(pressureVal) && pressureVal !== 255;
    var humidity2Val = parseFloat(configuration["humidity2"]);
    var humidity2Valid = !Number.isNaN(humidity2Val) && humidity2Val !== 255 && humidity2Val !== 0;
    var temperature2Val = parseFloat(configuration["temperature2"]);
    var temperature2Valid = !Number.isNaN(temperature2Val) && temperature2Val !== 255 && temperature2Val !== 0;
    $('.pressure-holder').toggleClass('hidden', !pressureValid);
    $('.humidity-holder2').toggleClass('hidden', !humidity2Valid);
    $('.temperature-holder2').toggleClass('hidden', !temperature2Valid);
    $('#pressure').html(pressureValid ? round(pressureVal,1) : '--');
    $('#humidity').html(round(configuration["humidity"],1));
    $('#humidity2').html(humidity2Valid ? round(configuration["humidity2"],1) : '--');
    $('#temperature').html(getTemperature(configuration["temperature"]));
    $('#temperature2').html(temperature2Valid ? getTemperature(configuration["temperature2"]) : '--');
    $('#temperature, #temperature2').toggleClass('fahrenheit',configuration["tempCF"]);

    // Keep Manual Override switch in sync with effective Day/Night status (indicator behavior)
    // UI: left=DAY, right=NIGHT => checked means NIGHT
    if (configuration["dayNightIsNight"] !== undefined) {
        $('#manualOverride').prop('checked', !!configuration["dayNightIsNight"]);
    }

    updateCathodeProtectButton();
    updateMaxLedCurrentInfo();
}

function updateCathodeProtectButton(){
    var isRunning = !!configuration["cathodeProtRunning"];
    $('#cath-prot-btn').text(isRunning ? 'Stop Cathode Protect' : 'Cathode Protect');
}

function updateMaxLedCurrentInfo(){
    var appliedCurrent = parseInt(configuration["ledCurrentmA"], 10);
    var appliedBrightness = parseInt(configuration["ledAppliedBrightness"], 10);
    var limitmA = parseInt(configuration["maxLedmA"], 10);
    var $info = $('#maxLedCurrentInfo');

    if(isNaN(limitmA)){
        limitmA = parseInt(configuration["ledLimitmA"], 10);
    }

    if(isNaN(limitmA)) limitmA = 0;
    if(isNaN(appliedCurrent)) appliedCurrent = 0;
    if(isNaN(appliedBrightness)) appliedBrightness = 0;

    var txt = 'Estimated LED current: ' + appliedCurrent + ' mA';
    txt += ' | Limit: ' + limitmA + ' mA';
    txt += ' | Applied brightness: ' + appliedBrightness;
    $info.text(txt);

    $info.removeClass('mqtt-badge mqtt-badge-fresh mqtt-badge-aging mqtt-badge-stale mqtt-badge-nodata info-muted');
    if(limitmA <= 0){
        $info.addClass('info-muted');
        return;
    }

    var ratio = appliedCurrent / limitmA;
    if(ratio >= 0.9){
        $info.addClass('mqtt-badge mqtt-badge-stale');
    }
    else if(ratio >= 0.75){
        $info.addClass('mqtt-badge mqtt-badge-aging');
    }
    else{
        $info.addClass('mqtt-badge mqtt-badge-fresh');
    }
}

function round(value, decimals) {
    return parseFloat(value).toFixed(decimals);
    //return Number(Math.round(value+'e'+decimals)+'e-'+decimals);
}

function getTemperature(temperatureInC){
    if(configuration["tempCF"]){        //Fahrenheit
        return round(temperatureInC * 1.8 + 32,1);
    }
    else{
        return round(temperatureInC,1);
    }
}

function getControlInfo(index){
    return controlInfos[index];
}

//Contains the most important initializes
function Init(){

    $('#dayBright, #nightBright').attr('max',configuration['maxBrightness']);
    document.title = configuration['version'];

    applyMenuAccessMode();



// Fixed RGB picker + sliders (new)
function applyRgbInputs(r,g,b){
    if($('#rgbFixR').length) $('#rgbFixR').val(r);
    if($('#rgbFixG').length) $('#rgbFixG').val(g);
    if($('#rgbFixB').length) $('#rgbFixB').val(b);
    if($('#rgbFixPicker').length) $('#rgbFixPicker').val(rgbToHex(r,g,b));
    if($('#rgbFixRval').length) $('#rgbFixRval').text(r);
    if($('#rgbFixGval').length) $('#rgbFixGval').text(g);
    if($('#rgbFixBval').length) $('#rgbFixBval').text(b);
}

$('#rgbFixPicker').on('input', function(){
    // Only update UI preview while dragging
    var rgb = hexToRgb($(this).val());
    if(!rgb) return;
    applyRgbInputs(rgb.r, rgb.g, rgb.b);
});

$('#rgbFixPicker').on('change', function(){
    var rgb = hexToRgb($(this).val());
    if(!rgb) return;

    configuration["rgbFixR"] = rgb.r;
    configuration["rgbFixG"] = rgb.g;
    configuration["rgbFixB"] = rgb.b;

    // LED preview instant
    sendMsgToArduino('rgbFixR', rgb.r);
    sendMsgToArduino('rgbFixG', rgb.g);
    sendMsgToArduino('rgbFixB', rgb.b);

    // EEPROM safe commit
    commitRgbDebounced(rgb.r, rgb.g, rgb.b);

    setPreviewColor();
});

$('#rgbFixR, #rgbFixG, #rgbFixB').on('input', function(){
    var r = parseInt($('#rgbFixR').val()||0,10);
    var g = parseInt($('#rgbFixG').val()||0,10);
    var b = parseInt($('#rgbFixB').val()||0,10);
    applyRgbInputs(r,g,b);
});

$('#rgbFixR, #rgbFixG, #rgbFixB').on('change', function(){
    var r = parseInt($('#rgbFixR').val()||0,10);
    var g = parseInt($('#rgbFixG').val()||0,10);
    var b = parseInt($('#rgbFixB').val()||0,10);

    configuration["rgbFixR"] = r;
    configuration["rgbFixG"] = g;
    configuration["rgbFixB"] = b;

    // LED preview instant
    sendMsgToArduino('rgbFixR', r);
    sendMsgToArduino('rgbFixG', g);
    sendMsgToArduino('rgbFixB', b);

    // EEPROM safe commit
    commitRgbDebounced(r,g,b);

    setPreviewColor();
});

// Fixed RGB default button
$('#btnRgbFixDefault').on('click', function(){
    var r = 255, g = 255, b = 255;
    configuration["rgbFixR"] = r;
    configuration["rgbFixG"] = g;
    configuration["rgbFixB"] = b;
    if($('#rgbFixR').length) $('#rgbFixR').val(r);
    if($('#rgbFixG').length) $('#rgbFixG').val(g);
    if($('#rgbFixB').length) $('#rgbFixB').val(b);
    if($('#rgbFixPicker').length) $('#rgbFixPicker').val(rgbToHex(r,g,b));
    // Commit the defaults
    sendMsgToArduino('rgbFixR', r);
    sendMsgToArduino('rgbFixG', g);
    sendMsgToArduino('rgbFixB', b);
    setPreviewColor();
    setTimeout(updateMaxLedCurrentInfo, 150);
});

$('#maxLedmA').on('input change', function(){
    var v = parseInt($(this).val(), 10);
    if(Number.isNaN(v)) v = 0;
    configuration["maxLedmA"] = v;
    configuration["ledLimitmA"] = v;
    updateMaxLedCurrentInfo();

    clearTimeout(_maxLedCommitTimer);
    _maxLedCommitTimer = setTimeout(function(){
        sendMsgToArduino('maxLedmA', v);
    }, 250);
});

    var sliderFadeOutTimer;
    function setFadeOutTimer(element){
        setTimeout(function(){
            if(isMouseDown > 0){
                clearTimeout(sliderFadeOutTimer);
                sliderFadeOutTimer = setFadeOutTimer(element);
            }
            else{
                $(element).addClass('fading').fadeOut('slow',function(){
                    $(this).remove();
                });
            }
        },600);
    }

    $('input[type="range"]').on('mousedown touchstart input',function(){
        clearTimeout(sliderFadeOutTimer);
        var id = $(this).attr('id');
        var parent = $(this).closest('.control-holder');
        $('[sliderfor]:not(.fading)').each(function(){
            if($(this).attr('sliderfor') != id && !$(this).hasClass('fading')){
                $(this).addClass('fading').fadeOut('slow',function(){
                    $(this).remove();
                });
            }
        });
        if($(parent).children('.slider-info').length == 0){
            $(parent).append('<div class="slider-info" sliderfor="'+id+'"></div>');
            $(parent).children('.slider-info').fadeIn(300);
        }
        var _this = $(parent).children('.slider-info').text($(this).val());
        sliderFadeOutTimer = setFadeOutTimer(_this);
    });

    $('.form label').on('click',function(){
        var id = '';
        if(!!$(this).attr('for')){
            id = $(this).attr('for');
        }
        else{
            id = $(this).closest('.control-holder').find('input, select, .info').eq(0).attr('id');
        }
        
        var info = getControlInfo(id);
        if(!!info){
            showPopUp($(this).text(), info, 450, true);
        }        
    });
    $('#popup-close-btn, #info-popup').on('click',function(){
        if($('#info-popup').hasClass('cantClose')){
            return;
        }
        else{
            $('#info-popup').fadeOut(450);
        }
    });
    $("#popup-box").click(function(event){
        event.stopPropagation();
    });

    //binds custom switch functionality
    $('.switcher').on('click',function(){
        $('#'+$(this).attr('for')).prop('checked',!$('#'+$(this).attr('for')).is(":checked")).trigger('change');
    });
    
    //binds custom switch text functionality
    $('.switcher-text').on('click',function(){
        $('#'+$(this).attr('for')).prop('checked',$(this).hasClass('right')).trigger('change');
    });

    //fills autogenerated select drop downs
    /*
    Possible css class:
    two-digit   - fills with two digit values if value lower than 10
    Possible attributes:
    min    (mandatory)    - auto generation's minimum value
    max    (mandatory)    - auto generation's maximum value
    step   (optional)     - auto generation's step, if not set, default is 1
    prefix (optional)     - concatanate this string before value
    suffix (optional)     - concatanate this string after value
    */
    $('.number-select').each(function(){
        var from = $(this).attr('min')*1;
        var to = $(this).attr('max')*1;
        var step = !!$(this).attr('step') ? $(this).attr('step') : 1;
        
        for(var i = from; i < to+1; i){
            var value = $(this).hasClass('two-digit') ? formatToTwoDigit(i) : i;
            var text_value = value;
            if($(this).attr('prefix') != null){
                text_value = $(this).attr('prefix') + text_value
            }
            if($(this).attr('suffix') != null){
                text_value += $(this).attr('suffix')
            }
            $(this).append('<option value="'+value+'">'+text_value+'</option>');
            i += step*1;
        }        
    });

    //fills the inputs with configuration values
    for(var index in configuration){
        var value = configuration[index];
        if ((index == 'touchShortAction' || index == 'touchDoubleAction' || index == 'touchLongAction') && String(value) === '3') {
            value = 4;
            configuration[index] = 4;
        }
        if(index == 'dayTime' || index == 'nightTime' || index == 'alarmTime'){
            value = value.split(':');
            $('#'+index+'Hours').val(formatToTwoDigit(value[0]));
            $('#'+index+'Minutes').val(formatToTwoDigit(value[1]));
        }
        else if(index == 'version'
		){
            $('#'+index).html(value);
        }
        else if(index == 'utc_offset' || index == 'maxBrightness' || 
                index == 'dayBright' || index == 'nightBright' || 
                index == 'animMode' || index == 'rgbBrightness' ||
                index == 'maxLedmA' ||
                index == 'rgbSpeed' ||
                index == 'rgbEffect' || index == 'interval' ||
				index == 'alarmPeriod' || index == 'radarTimeout' ||
				index == 'dateRepeatMin' || index == 'dateMode' ||
				index == 'dateStart' || index == 'dateEnd' ||
				index == 'timeStart' || index == 'timeEnd' ||
				index == 'tempStart' || index == 'tempEnd' ||
				index == 'humidStart' || index == 'humidEnd' ||
                index == 'pressureStart' || index == 'pressureEnd' ||
				index == 'mqttBrokerRefresh' || index == 'tempRepeatMin' ||
				index == 'wifiSsid' || index == 'wifiPsw' || 
				index == 'ApSsid' || index == 'ApPsw' || 
				index == 'mqttBrokerAddr' || index == 'NtpServer' ||
				index == 'mqttBrokerUser' || index =='mqttBrokerPsw' ||
				index == 'firmware' || index == 'corrT0' || index == 'corrT1' ||
                index == 'corrH0' || index == 'corrH1' || index == 'cathProtMin' ||
                index == 'touchShortAction' || index == 'touchDoubleAction' || index == 'touchLongAction' ||
                index == 'gestureUpAction' || index == 'gestureDownAction' || index == 'gestureLeftAction' || index == 'gestureRightAction'
            ){
            $('#'+index).val(value);
        }
        else if((index == 'enableDST' || index == 'set12_24' ||
                index == 'showZero' || index == 'enableBlink' ||
                index == 'enableAutoShutoff' || index == 'alarmEnable' ||
                index == 'rgbDir' || index == 'manualOverride' ||
				index == 'enableAutoDim' || index == 'enableRadar' ||
				index == 'enableDoubleBlink' || index == 'enableTimeDisplay' ||
                index == 'enableTempDisplay' || index == 'enableHumidDisplay' || index == 'enablePressDisplay' ||
                index == 'mqttEnable' || index == 'tempCF' || index == "wifiMode" || index == 'tubesSleep' || index == 'onboardLed'
                ) && !!value
            ){
            $('#'+index).prop('checked',true);
        }
        else if(index == "maxDigits"){
            //TODO
        }
    }
	$('.radar-holder').toggleClass('hidden',configuration['radarTimeout'] == 0);
	$('.mqtt-holder').toggleClass('hidden',configuration['mqttBrokerRefresh'] == 0);
	$('.lux-holder').toggleClass('hidden',configuration['lux'] == 255);
    $('.pressure-holder').toggleClass('hidden',configuration['pressure'] == 255);
	$('.humidity-holder').toggleClass('hidden',configuration['humidity'] == 255);
    $('.humidity-holder2').toggleClass('hidden',(configuration['humidity2'] == 255) || (parseFloat(configuration['humidity2']) === 0));
    $('.temperature-holder').toggleClass('hidden',configuration['temperature'] == 255);
    $('.temperature-holder2').toggleClass('hidden',(configuration['temperature2'] == 255) || (parseFloat(configuration['temperature2']) === 0));
    $('.rgb-holder').toggleClass('hidden',configuration['rgbEffect'] == 255);
	    $('#fixedColorRgbSection').toggleClass('hidden', parseInt(configuration['rgbEffect'],10) !== 1);
$('.tube-holder').toggleClass('hidden',configuration['tubeDriver'] == "DUMMY");
	$('.wordclock-holder').toggleClass('hidden',configuration['tubeDriver'] == "WORDCLOCK");
	$('.sensors-holder').toggleClass('hidden',configuration['temperature'] == 255);
    $('.gesture-holder').toggleClass('hidden', !configuration['gestureSensorPresent']);

    // Firmware-specific UI: hide unsupported "Transition" tube animation for fw64.
    (function applyFirmwareSpecificUi(){
        var fw = String(configuration['FW'] || '').toLowerCase();
        var hideTransition = (fw === 'fw64');
        var $transitionOption = $('#animMode option[value="5"]');
        if($transitionOption.length){
            $transitionOption.prop('hidden', hideTransition).prop('disabled', hideTransition);
            if(hideTransition && String($('#animMode').val()) === '5'){
                $('#animMode').val('0');
            }
        }
    })();

    setPreviewColor();
    initMqttDevicesUI();

    //sets a possible good timezone, if not already set
    //if(!configuration['utc_offset']){
    //    //Intl.DateTimeFormat().resolvedOptions().timeZone <- I am in this Zone
    //    var tryToFindCurrentZone = -(new Date().getTimezoneOffset() / 60);
    //    $('#utc_offset').val(tryToFindCurrentZone);
    //}

    setTimeout(function(){
        $('input, select').on('change',function(){
            var controlId = $(this).attr('id');
            if(controlId === 'maxLedmA'){
                return;
            }
            var value = '';
            // Time fields are split into <key>Hours and <key>Minutes. Only post the combined key once.
            if(controlId.indexOf("Hours") == -1 && controlId.indexOf("Minutes") == -1){
                if($(this).attr('type') == 'checkbox'){
                    value = $(this).is(':checked');
                }
                else{
                    value = $(this).val();
                }
                sendMsgToArduino(controlId, value);
                if(controlId === 'rgbEffect'){
                    $('#fixedColorRgbSection').toggleClass('hidden', parseInt(value,10) !== 1);
                }
            }
            else{
                var key = controlId.replace('Hours','').replace('Minutes','');
                // Send both fields separately to match firmware
                var hours = $('#'+key+'Hours').val();
                var minutes = $('#'+key+'Minutes').val();
                sendMsgToArduino(key + 'Hours', hours);
                sendMsgToArduino(key + 'Minutes', minutes);
            }
        });
        $('#dateMode').on('change',function(){
            configuration["dateMode"] = $(this).val();
            setCurrentInfos();
        });
        $('#tempCF').on('change',function(){
            configuration["tempCF"] = !configuration["tempCF"];
            setCurrentInfos();
        });
    },200);
}

function showPopUp(header, content, timeout, canClose){
    $('#popup-box h2').text(header);
    $('#popup-box .content').html(content);
    $('#info-popup').toggleClass('cantClose',!canClose);
    $('#info-popup').fadeIn(timeout);
}

function areYouSure(fn){
    if (confirm("Are you sure to run " + fn + "?")) {
        if(fn == "reset"){
            callReset();
        }
        else if(fn == "factoryreset"){
            callFactoryReset();
        }
        else if(fn == "firmwareupdate"){
            callFirmwareUpdate();
        }
        else if(fn == "cathodeProtect"){
            callCathodeProtect();
        }		
    }
}


function callReset(){
    if(isTest){return;}
    apiPost('/reset/', {}).done(function(data){
        location.reload();
    }).always(function(){
        
    });
}

function callFactoryReset(){
    if(isTest){return;}
    apiPost('/factoryreset/', {}).done(function(data){
        location.reload();
    }).always(function(){
        
    });
}

function callFirmwareUpdate(){
    if(isTest){return;}
    showPopUp("Firmware is updating", "Please do not turn off the clock", 300, false);
    apiPost('/firmwareupdate', {}).done(function(data){
        if(!!data.header){
            showPopUp(data.header, data.content, 300, data.canClose);
        }
    }).always(function(){
        
    });
}

function callCathodeProtect(){
    if(isTest){return;}
    apiPost('/cathodeProtect', {}).done(function(data){
        if(data && data.cathodeProtRunning !== undefined){
            configuration["cathodeProtRunning"] = !!data.cathodeProtRunning;
            updateCathodeProtectButton();
        }
        if(!!data.header){
            showPopUp(data.header, data.content, 300, data.canClose);
        }
    }).always(function(){
        
    });
}

function getClockDetails(){
    if(isTest){return;}
    apiGet('/getClockDetails/').done(function(data){
        $('#clock-details').html(data);
    }).always(function(){
        
    });
}

function getCurrentInfos(){
    if(isTest){return;}
    $.get('/getCurrentInfos').done(function(data){
        for(var i in data){
            configuration[i] = data[i];
        }
        updateMainExtrasFromCurrentInfos(data);
        if(!!data["popupMsg"] && data["popupMsg"].length > 0){
            showPopUp("Info", data["popupMsg"], 300, true);
        }
        setCurrentInfos();
        setTimeout(getCurrentInfos,20000);   //refreshes time every 20 second by calling itself
    }).always(function(){
        
    });
}

function getCurrentDate(date){
    var splittedDate = date.split('.');
    var yyyy = splittedDate[0];
    var mm = splittedDate[1]; 
    var dd = splittedDate[2];

    if(configuration["dateMode"] == 0){
        return dd+"/"+mm+"/"+yyyy;
    }
    else if(configuration["dateMode"] == 1){
        return mm+"/"+dd+"/"+yyyy;
    }
    else if(configuration["dateMode"] == 2){
        return yyyy+"/"+mm+"/"+dd;
    }
    else{
        return yyyy+"/"+mm+"/"+dd;   //if dateMode?? // 0:dd/mm/yyyy 1:mm/dd/yyyy 2:yyyy/mm/dd
    }
}

function setDateTime() {
    var manualDateTime = String($('#manualDateTime').val() || '').trim();
    var parts = manualDateTime.split('T');
    var manualDate = parts[0] || '';
    var manualTime = (parts[1] || '').slice(0,5);

    if (manualDate && manualTime) {
        sendMsgToArduino('currentDate', manualDate);
        sendMsgToArduino('currentTime', manualTime);
        alert('Date and Time set successfully!');
    } else {
        alert('Please select both date and time.');
    }
}


//Gets the current time from browser and gives back as readable format
function getCurrentTime(){
    var today = new Date();
    var dd = formatToTwoDigit(today.getDate());
    var mm = formatToTwoDigit(today.getMonth()+1); 
    var yyyy = today.getFullYear();
    var hour = formatToTwoDigit(today.getHours());
    var minute = formatToTwoDigit(today.getMinutes());

    if(configuration["dateMode"] == 0){
        return dd+"-"+mm+"-"+yyyy+" "+hour+":"+minute;
    }
    else if(configuration["dateMode"] == 1){
        return mm+"-"+dd+"-"+yyyy+" "+hour+":"+minute;
    }
    else if(configuration["dateMode"] == 2){
        return yyyy+"-"+mm+"-"+dd+" "+hour+":"+minute;
    }
    else{
        return yyyy+"-"+mm+"-"+dd+" "+hour+":"+minute;   //if dateMode?? // 0:dd/mm/yyyy 1:mm/dd/yyyy 2:yyyy/mm/dd
    }
}

//if number is lower than 10, adds a zero
function formatToTwoDigit(number){
    return ("0" + number).slice(-2);
}
