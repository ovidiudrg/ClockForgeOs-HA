// System Info Functions
var _sysCpuTempSeries = [];
var _sysNetBwSeries = [];
var _sysLastSampleMs = 0;
var _sysPollTimer = null;
var _sysPollIntervalMs = 10000;
var _sysMaxPoints = 30;
var _sysWifiSwitchLastResult = '';
var _sysWifiSwitchLastChangeMs = 0;

function _sysIsPageActive() {
    return $('[page="system"]').hasClass('active');
}

function _sysStartPolling() {
    if (_sysPollTimer) return;
    _sysPollTimer = setInterval(function(){
        if (_sysIsPageActive() && !document.hidden) {
            fetchSystemInfo();
        }
    }, _sysPollIntervalMs);
}

function _sysStopPolling() {
    if (_sysPollTimer) {
        clearInterval(_sysPollTimer);
        _sysPollTimer = null;
    }
}

function _sysPushSeries(series, value, maxLen) {
    if (typeof value !== 'number' || Number.isNaN(value)) return;
    series.push(value);
    if (series.length > maxLen) {
        series.splice(0, series.length - maxLen);
    }
}

function _sysDrawTrend(canvasId, series, color) {
    var canvas = document.getElementById(canvasId);
    if (!canvas || !canvas.getContext) return;
    var ctx = canvas.getContext('2d');
    var width = canvas.width;
    var height = canvas.height;
    ctx.clearRect(0, 0, width, height);

    ctx.strokeStyle = 'rgba(120,120,120,0.35)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(0, height - 1);
    ctx.lineTo(width, height - 1);
    ctx.stroke();

    if (!series || series.length < 2) return;

    var min = Math.min.apply(null, series);
    var max = Math.max.apply(null, series);
    if (max - min < 0.001) {
        max = min + 1;
    }

    var pad = 6;
    var plotH = height - pad * 2;
    var stepX = (width - pad * 2) / (series.length - 1);

    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.beginPath();
    for (var i = 0; i < series.length; i++) {
        var v = series[i];
        var x = pad + i * stepX;
        var y = pad + (max - v) * plotH / (max - min);
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
    }
    ctx.stroke();
}

function fetchSystemInfo() {
    $.get('/getSystemInfo').done(function(data) {
        var sampleBytes = 0;
        var sampleMs = Date.now();
        if (typeof data === 'string') {
            sampleBytes = data.length;
            try { data = JSON.parse(data); } catch(e) { console.log('Invalid JSON from /getSystemInfo'); }
        }
        // keep lightweight: when jQuery already parsed JSON object, skip costly re-stringify.
        if (sampleBytes <= 0) sampleBytes = 512;
        updateSystemInfoDisplay(data || {}, sampleBytes, sampleMs);
    }).fail(function() {
        console.log('Failed to fetch system info');
    });
}

function updateSystemInfoDisplay(data, sampleBytes, sampleMs) {
    function esc(v){
        return String(v === undefined || v === null ? '' : v)
            .replace(/&/g,'&amp;')
            .replace(/</g,'&lt;')
            .replace(/>/g,'&gt;')
            .replace(/\"/g,'&quot;')
            .replace(/'/g,'&#39;');
    }
    function formatPhysicalSensors(v){
        var raw = String(v === undefined || v === null ? '' : v).trim();
        if(!raw || raw === '--') return '--';

        var parts = raw
            .split(/[;,|]+/)
            .map(function(x){ return x.trim(); })
            .filter(function(x){ return x.length > 0; });

        if(parts.length <= 1) return raw.replace(/\s{2,}/g, ' ');
        return parts.join(' • ');
    }
    function formatUptimeHM(dataObj){
        if (dataObj && dataObj.uptimeMinutes !== undefined) {
            var totalMin = parseInt(dataObj.uptimeMinutes, 10);
            if (!Number.isNaN(totalMin) && totalMin >= 0) {
                var h = Math.floor(totalMin / 60);
                var m = totalMin % 60;
                return h + ':' + (m < 10 ? '0' : '') + m;
            }
        }
        if (dataObj && dataObj.uptime !== undefined) {
            var legacyHours = parseInt(dataObj.uptime, 10);
            if (!Number.isNaN(legacyHours) && legacyHours >= 0) {
                return legacyHours + ':00';
            }
        }
        return '--';
    }

    $('#sysOsVersion').text((data && data.osVersion) ? data.osVersion : '--');
    $('#sysFirmwareID').text((data && data.firmwareID) ? data.firmwareID : '--');
    $('#sysChipModel').text((data && data.chipModel) ? data.chipModel : '--');
    $('#sysTubeDriver').text((data && data.tubeDriver) ? data.tubeDriver : '--');
    $('#sysCpuTemp').text((data && data.cpuTemp !== undefined) ? (data.cpuTemp + ' °C') : '--');
    $('#sysCpuCores').text((data && data.cpuCores !== undefined) ? data.cpuCores : '--');
    $('#sysCpuFreq').text((data && data.cpuFreqMHz !== undefined) ? (data.cpuFreqMHz + ' MHz') : '--');
    $('#sysUptime').text(formatUptimeHM(data));
    if (data && data.freeHeap !== undefined && data.totalHeap !== undefined) {
        var usedPct = (data.heapUsedPercent !== undefined) ? data.heapUsedPercent : '--';
        $('#sysHeap').text(formatBytes(data.freeHeap) + ' free / ' + formatBytes(data.totalHeap) + ' total (' + usedPct + '% used)');
    } else {
        $('#sysHeap').text('--');
    }
    $('#sysMacAddr').text((data && data.macAddress) ? data.macAddress : '--');
    $('#sysMaxBrightness').text((data && data.maxBrightness !== undefined) ? data.maxBrightness : '--');
    var physicalSensorsText = (data && data.physicalSensors !== undefined)
        ? data.physicalSensors
        : ((data && data.installedSensors !== undefined) ? data.installedSensors : '--');
    $('#sysPhysicalSensors').text(formatPhysicalSensors(physicalSensorsText));
    $('#sysVirtualSensors').text((data && data.virtualSensors !== undefined) ? data.virtualSensors : '--');
    $('#sysTempSensors').text((data && data.temperatureSensors !== undefined) ? data.temperatureSensors : '--');
    $('#sysHumidSensors').text((data && data.humiditySensors !== undefined) ? data.humiditySensors : '--');
    $('#sysPressSensors').text((data && data.pressureSensors !== undefined) ? data.pressureSensors : '--');
    $('#sysWifiSignal').text((data && data.wifiSignal !== undefined) ? (data.wifiSignal + ' dBm') : '--');
    $('#sysWifiStatus').text((data && data.wifiStatus !== undefined) ? data.wifiStatus : '--');
    var wifiSwitchText = '--';
    if (data) {
        var switchResult = data.wifiSwitchResult || 'idle';
        var switchTarget = data.wifiSwitchTargetSsid || '';
        var switchRollback = data.wifiSwitchRollbackSsid || '';

        if (switchResult !== _sysWifiSwitchLastResult) {
            _sysWifiSwitchLastResult = switchResult;
            _sysWifiSwitchLastChangeMs = sampleMs || Date.now();
        }

        if (switchResult === 'connecting' || data.wifiSwitchPending) {
            wifiSwitchText = switchTarget ? ('Connecting to ' + switchTarget) : 'Connecting...';
        } else if (switchResult === 'rollback_started') {
            wifiSwitchText = switchRollback ? ('Rollback to ' + switchRollback) : 'Rollback started';
        } else if (switchResult === 'rollback_connected') {
            wifiSwitchText = switchRollback ? ('Restored: ' + switchRollback) : 'Rollback restored';
            if (_sysWifiSwitchLastChangeMs > 0) {
                var ageSec = Math.max(0, Math.floor(((sampleMs || Date.now()) - _sysWifiSwitchLastChangeMs) / 1000));
                wifiSwitchText += ' (' + ageSec + 's ago)';
            }
        } else if (switchResult === 'connect_failed') {
            wifiSwitchText = switchTarget ? ('Failed: ' + switchTarget) : 'Connect failed';
        } else if (switchResult === 'connected') {
            wifiSwitchText = switchTarget ? ('Connected: ' + switchTarget) : 'Connected';
        } else {
            var currentSsid = data.wifiSsid || '';
            wifiSwitchText = currentSsid ? ('Idle (' + currentSsid + ')') : 'Idle';
        }
    }
    $('#sysWifiSwitch').text(wifiSwitchText);
    $('#sysWifiIP').text((data && data.wifiIP !== undefined) ? data.wifiIP : '--');
    $('#sysMqttStatus').text((data && data.mqttStatus !== undefined) ? data.mqttStatus : '--');

    var cpuTemp = parseFloat(data && data.cpuTemp);
    if (!Number.isNaN(cpuTemp)) {
        _sysPushSeries(_sysCpuTempSeries, cpuTemp, _sysMaxPoints);
        $('#sysCpuTempTrendVal').text(cpuTemp.toFixed(1) + ' °C');
        _sysDrawTrend('sysCpuTempChart', _sysCpuTempSeries, '#37bfb9');
    }

    if (typeof sampleBytes === 'number' && sampleBytes > 0) {
        if (_sysLastSampleMs > 0 && sampleMs > _sysLastSampleMs) {
            var sec = (sampleMs - _sysLastSampleMs) / 1000.0;
            var bytesPerSec = sampleBytes / sec;
            _sysPushSeries(_sysNetBwSeries, bytesPerSec, _sysMaxPoints);
            $('#sysNetBwTrendVal').text((bytesPerSec / 1024.0).toFixed(2) + ' KB/s');
            _sysDrawTrend('sysNetBwChart', _sysNetBwSeries, '#D96025');
        }
        _sysLastSampleMs = sampleMs;
    }

    var pinsHtml = '';
    if (Array.isArray(data.usedPins)) {
        var pins = data.usedPins.slice().sort(function(a,b){ return (a.num||0) - (b.num||0); });
        if (pins.length > 0) {
            pinsHtml += '<table class="sys-table">';
            pinsHtml += '<thead><tr><th>GPIO</th><th>Function</th></tr></thead><tbody>';
            pins.forEach(function(pin) {
                pinsHtml += '<tr><td>' + esc(pin.num) + '</td><td>' + esc(pin.label) + '</td></tr>';
            });
            pinsHtml += '</tbody></table>';
        }
    }
    $('#sysUsedPins').html(pinsHtml || '--');

    var driverPinsHtml = '';
    var driverLabel = 'Driver Pin Settings';
    if (Array.isArray(data.hv5122Pins)) {
        if (data.hv5122Pins.length > 0) {
            driverLabel = 'HV5122 Pin Settings';
            driverPinsHtml += '<table class="sys-table sys-hv-table">';
            driverPinsHtml += '<thead><tr><th>Tube</th>';
            for (var c = 0; c < 10; c++) driverPinsHtml += '<th>D' + c + '</th>';
            driverPinsHtml += '</tr></thead><tbody>';
            data.hv5122Pins.forEach(function(row, idx) {
                driverPinsHtml += '<tr><td>T' + idx + '</td>';
                for (var j = 0; j < 10; j++) {
                    var cell = (Array.isArray(row) && row[j] !== undefined) ? row[j] : '';
                    driverPinsHtml += '<td>' + esc(cell) + '</td>';
                }
                driverPinsHtml += '</tr>';
            });
            driverPinsHtml += '</tbody></table>';
        }
    }
    else if (Array.isArray(data.max6921SegmentPins) || Array.isArray(data.max6921DigitPins)) {
        driverLabel = 'MAX6921 Pin Settings';

        var seg = Array.isArray(data.max6921SegmentPins) ? data.max6921SegmentPins : [];
        var dig = Array.isArray(data.max6921DigitPins) ? data.max6921DigitPins : [];
        var ctrl = (data && typeof data.max6921ControlPins === 'object' && data.max6921ControlPins) ? data.max6921ControlPins : null;

        if (ctrl) {
            driverPinsHtml += '<div class="info-row"><span class="label">LE:</span><span class="value mono-text">GPIO' + esc(ctrl.PIN_LE) + '</span></div>';
            driverPinsHtml += '<div class="info-row"><span class="label">CLK:</span><span class="value mono-text">GPIO' + esc(ctrl.PIN_CLK) + '</span></div>';
            driverPinsHtml += '<div class="info-row"><span class="label">DATA:</span><span class="value mono-text">GPIO' + esc(ctrl.PIN_DATA) + '</span></div>';
            driverPinsHtml += '<div class="info-row"><span class="label">BL:</span><span class="value mono-text">GPIO' + esc(ctrl.PIN_BL) + '</span></div>';
        }

        if (seg.length > 0) {
            driverPinsHtml += '<table class="sys-table"><thead><tr><th>Segment</th><th>OUT bit</th></tr></thead><tbody>';
            var segNames = ['a','b','c','d','e','f','g','dp'];
            for (var si = 0; si < seg.length; si++) {
                var sn = (si < segNames.length) ? segNames[si] : ('s' + si);
                driverPinsHtml += '<tr><td>' + esc(sn) + '</td><td>' + esc(seg[si]) + '</td></tr>';
            }
            driverPinsHtml += '</tbody></table>';
        }

        if (dig.length > 0) {
            driverPinsHtml += '<table class="sys-table"><thead><tr><th>Digit</th><th>OUT bit</th></tr></thead><tbody>';
            for (var di = 0; di < dig.length; di++) {
                driverPinsHtml += '<tr><td>' + esc(di + 1) + '</td><td>' + esc(dig[di]) + '</td></tr>';
            }
            driverPinsHtml += '</tbody></table>';
        }
    }
    else if (data && data.driverSetupStr) {
        driverPinsHtml = String(data.driverSetupStr);
    }

    $('#sysDriverPinsLabel').text(driverLabel);
    $('#sysDriverPins').html(driverPinsHtml || '--');
}

function formatBytes(bytes) {
    if (bytes === 0) return '0 B';
    var k = 1024;
    var sizes = ['B', 'KB', 'MB', 'GB'];
    var i = Math.floor(Math.log(bytes) / Math.log(k));
    return Math.round(bytes / Math.pow(k, i) * 100) / 100 + ' ' + sizes[i];
}

function initSystemInfo() {
    _sysCpuTempSeries = [];
    _sysNetBwSeries = [];
    _sysLastSampleMs = 0;

    $(document).on('click', '[pagemenu="system"]', function() {
        fetchSystemInfo();
        _sysStartPolling();
    });

    document.addEventListener('visibilitychange', function(){
        if (document.hidden) {
            _sysStopPolling();
        } else if (_sysIsPageActive()) {
            fetchSystemInfo();
            _sysStartPolling();
        }
    });

    $(document).on('click', '[pagemenu]:not([pagemenu="system"])', function() {
        _sysStopPolling();
    });
    
    $('#sys-refresh-btn').on('click', function() {
        fetchSystemInfo();
    });
    
    if ($('[page="system"]').hasClass('active')) {
        fetchSystemInfo();
        _sysStartPolling();
    }
}
