// ===================== Clock64 Main Page extras =====================
var _manualDisplayOff = false;
var _wakeOnMotionEnabled = true;
var _wakeSaveTimer = null;

function pad2(n){ return (n<10?'0':'')+n; }

function toBool(v, fallback){
    if(v === undefined || v === null){
        return !!fallback;
    }
    if(typeof v === 'boolean') return v;
    if(typeof v === 'number') return v !== 0;
    var s = String(v).trim().toLowerCase();
    if(s === 'true' || s === '1' || s === 'on' || s === 'yes') return true;
    if(s === 'false' || s === '0' || s === 'off' || s === 'no') return false;
    return !!fallback;
}

function postSetting(key, value){
    if(isTest){ console.log("SET", key, value); return; }
    return apiPost('/saveSetting', { "key": key, "value": String(value) });
}

function postManualDateTime(dateStr, timeStr){
    if(isTest){ console.log("SET MANUAL TIME", dateStr, timeStr); return; }
    return apiPost('/setManualTime', { "date": dateStr, "time": timeStr });
}

function splitManualDateTime(v){
    var raw = String(v || '').trim();
    if(!raw) return null;
    var dt = raw.replace(' ', 'T');
    var parts = dt.split('T');
    if(parts.length < 2) return null;
    var d = parts[0];
    var t = parts[1].slice(0,5);
    if(!d || !t) return null;
    return { date: d, time: t };
}

function syncManualDisplayControl(){
    $("#manualDisplayPower").prop("checked", !_manualDisplayOff);
    if($("#btnToggleDisplay").length){
        $("#btnToggleDisplay").text(_manualDisplayOff ? "Turn display ON" : "Turn display OFF");
    }
}

function applyConfigToMainExtras(cfg){
    if(cfg.wakeOnMotionEnabled !== undefined){
        _wakeOnMotionEnabled = toBool(cfg.wakeOnMotionEnabled, true);
        $("#wakeOnMotionEnabled").prop("checked", _wakeOnMotionEnabled);
    }
    if(cfg.tubesWakeSeconds !== undefined){
        $("#tubesWakeSeconds").val(cfg.tubesWakeSeconds);
    }
    if(cfg.manualDisplayOff !== undefined){
        _manualDisplayOff = toBool(cfg.manualDisplayOff, false);
        syncManualDisplayControl();
    }
}

function updateMainExtrasFromCurrentInfos(data){
    if(data.timeSource !== undefined){
        $("#timeSource").text(data.timeSource);
    }
    if(data.manualDisplayOff !== undefined){
        _manualDisplayOff = toBool(data.manualDisplayOff, false);
    }
    if(data.wakeOnMotionEnabled !== undefined){
        _wakeOnMotionEnabled = toBool(data.wakeOnMotionEnabled, true);
        $("#wakeOnMotionEnabled").prop("checked", _wakeOnMotionEnabled);
    }
    if(data.tubesWakeSeconds !== undefined){
        var active = (document.activeElement && document.activeElement.id === "tubesWakeSeconds");
        if(!active){
            $("#tubesWakeSeconds").val(data.tubesWakeSeconds);
        }
    }
    if(data.tubesPower !== undefined){
        var displayIsOn = toBool(data.tubesPower, true);
        $("#displayStatus").text(displayIsOn ? "ON" : "OFF");
        $("#displayStatus, #displayStatusDetail, #currentTime").toggleClass("display-off-muted", !displayIsOn);
    }
    if(data.telnetClients !== undefined){
        $("#telnetClients").text(data.telnetClients);
    }
    if(data.mqttClients !== undefined){
        $("#mqttClients").text(data.mqttClients);
    }
    if(data.mqttStatus !== undefined || data.mqttClients !== undefined){
        var connected = (data.mqttStatus === "Connected") || (parseInt(data.mqttClients || 0, 10) > 0);
        $("#mqttMainStatus")
            .text(connected ? "Active" : "Inactive")
            .toggleClass('mqtt-state-on', connected)
            .toggleClass('mqtt-state-off', !connected);
    }
    if (data.manualDisplayOff !== undefined && toBool(data.manualDisplayOff, false)) {
        $("#displayStatusDetail").text("Manual OFF");
    } else if (data.wakeOnMotionEnabled !== undefined && !data.wakeOnMotionEnabled) {
        $("#displayStatusDetail").text("Always ON");
    } else if (data.wakeSecondsLeft !== undefined && parseInt(data.wakeSecondsLeft,10) > 0) {
        $("#displayStatusDetail").text("Awake (" + data.wakeSecondsLeft + " s left)");
    } else {
        $("#displayStatusDetail").text("Sleeping");
    }
    syncManualDisplayControl();
}

function initMainExtrasBindings(){
    if($("#btnToggleDisplay").length){
        $("#btnToggleDisplay").on("click", function(){
            var wantOn = _manualDisplayOff;
            postSetting("displayPower", wantOn ? "true" : "false");
            _manualDisplayOff = !wantOn;
            syncManualDisplayControl();
        });
    }

    $("#manualDisplayPower").on("change", function(){
        var isOn = $("#manualDisplayPower").is(":checked");
        _manualDisplayOff = !isOn;
        postSetting("displayPower", isOn ? "true" : "false");
    });

    $("#wakeOnMotionEnabled").on("change", function(){
        var v = $("#wakeOnMotionEnabled").is(":checked");
        _wakeOnMotionEnabled = v;
        postSetting("wakeOnMotionEnabled", v ? "true" : "false");
    });

    $("#tubesWakeSeconds").on("input", function(){
        clearTimeout(_wakeSaveTimer);
        _wakeSaveTimer = setTimeout(function(){
            var v = parseInt($("#tubesWakeSeconds").val(), 10);
            if(isNaN(v)) v = 10;
            if(v < 1) v = 1;
            if(v > 3600) v = 3600;
            $("#tubesWakeSeconds").val(v);
            postSetting("tubesWakeSeconds", String(v));
        }, 700);
    });

    $("#btnSetManualDateTime").on("click", function(){
        var dt = splitManualDateTime($("#manualDateTime").val());
        if(!dt){
            alert("Please select date and time.");
            return;
        }
        postManualDateTime(dt.date, dt.time);
    });

    $("#btnSetBrowserDateTime").on("click", function(){
        var now = new Date();
        var d = now.getFullYear()+"-"+pad2(now.getMonth()+1)+"-"+pad2(now.getDate());
        var t = pad2(now.getHours())+":"+pad2(now.getMinutes());
        $("#manualDateTime").val(d+"T"+t);
        postManualDateTime(d, t);
    });
}
