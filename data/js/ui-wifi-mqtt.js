// WiFi scan & connect UI
function scanWifi() {
    $('#wifi-scan-results').html('<div class="info">Scanning...</div>');
    apiGet('/scanWifi').done(function(data) {
        if (typeof data === 'string') {
            try { data = JSON.parse(data); } catch(e){ data = {}; }
        }
        if (data && data.status === 'busy') {
            $('#wifi-scan-results').html('<div class="info">Busy: WiFi connect in progress. Try again in a few seconds.</div>');
            return;
        }
        if (data && data.status === 'blocked' && data.reason === 'ap_client_connected') {
            var cnt = (typeof data.clients === 'number') ? data.clients : 1;
            $('#wifi-scan-results').html('<div class="info">Scan paused to keep AP connection stable (' + cnt + ' AP client' + (cnt === 1 ? '' : 's') + '). Connect this device to normal WiFi first, then scan again.</div>');
            return;
        }
        if (data && data.status) {
            var attempts = 0;
            var poll = setInterval(function(){
                apiGet('/scanWifi').done(function(d2){
                    if (typeof d2 === 'string') {
                        try { d2 = JSON.parse(d2); } catch(e){ d2 = {}; }
                    }
                    if (Array.isArray(d2)) {
                        clearInterval(poll);
                        updateWifiScanResults(d2);
                        return;
                    }
                    if (d2 && d2.status === 'busy') {
                        clearInterval(poll);
                        $('#wifi-scan-results').html('<div class="info">Busy: WiFi connect in progress. Try again in a few seconds.</div>');
                        return;
                    }
                    if (d2 && d2.status === 'blocked' && d2.reason === 'ap_client_connected') {
                        var cnt2 = (typeof d2.clients === 'number') ? d2.clients : 1;
                        clearInterval(poll);
                        $('#wifi-scan-results').html('<div class="info">Scan paused to keep AP connection stable (' + cnt2 + ' AP client' + (cnt2 === 1 ? '' : 's') + '). Connect this device to normal WiFi first, then scan again.</div>');
                        return;
                    }
                    if (d2 && d2.status === 'scanning') {
                        attempts++;
                        if (attempts > 40) {
                            clearInterval(poll);
                            $('#wifi-scan-results').html('<div class="info">Scan timeout</div>');
                        }
                        return;
                    }
                    attempts++;
                    if (attempts > 20) {
                        clearInterval(poll);
                        $('#wifi-scan-results').html('<div class="info">Scan failed</div>');
                    }
                }).fail(function(){
                    clearInterval(poll);
                    $('#wifi-scan-results').html('<div class="info">Scan failed</div>');
                });
            }, 800);
        } else if (Array.isArray(data)) {
            updateWifiScanResults(data);
        } else {
            $('#wifi-scan-results').html('<div class="info">No results</div>');
        }
    }).fail(function(){
        $('#wifi-scan-results').html('<div class="info">Scan failed</div>');
    });
}

function updateWifiScanResults(list) {
    if (!list || list.length === 0) {
        $('#wifi-scan-results').html('<div class="info">No networks found</div>');
        return;
    }
    var html = '<table class="wifi-scan-table">';
    html += '<tr><th class="wifi-scan-header-left">SSID</th><th>RSSI</th><th></th></tr>';
    list.forEach(function(it){
        var ss = $('<div>').text(it.ssid).html();
        var rssi = it.rssi;
        var enc = it.enc;
        html += '<tr><td class="wifi-scan-cell-ssid">'+ss+'</td>';
        html += '<td class="wifi-scan-cell-rssi">'+rssi+'</td>';
        html += '<td class="wifi-scan-cell-actions"><button class="btn wifi-fill" data-ssid="'+encodeURIComponent(it.ssid)+'">Use</button>';
        if (enc == 0) html += '&nbsp;<button class="btn wifi-quick" data-ssid="'+encodeURIComponent(it.ssid)+'">Connect</button>';
        html += '</td></tr>';
    });
    html += '</table>';
    $('#wifi-scan-results').html(html);

    $('.wifi-fill').on('click', function(){
        var ss = decodeURIComponent($(this).attr('data-ssid'));
        $('#wifiSsid').val(ss);
        $('#wifiPsw').focus();
    });
    $('.wifi-quick').on('click', function(){
        var ss = decodeURIComponent($(this).attr('data-ssid'));
        connectToWifi(ss, '');
    });
}

function connectToWifi(ssid, psw) {
    var url = '/connectWifi?ssid=' + encodeURIComponent(ssid) + '&psw=' + encodeURIComponent(psw || '');
    apiGet(url).done(function(resp){
        if (typeof resp === 'string') {
            try { resp = JSON.parse(resp); } catch(e){ resp = {}; }
        }
        if (resp && resp.status === 'already_connected') {
            alert('Already connected to ' + (resp.ssid || ssid) + (resp.ip ? (' (' + resp.ip + ')') : ''));
            return;
        }
        if (resp && resp.status === 'busy') {
            alert('Another WiFi connect is already running. Please wait and try again.');
            return;
        }
        if (resp && resp.status === 'connecting') {
            alert('Connecting to ' + ssid + (resp.rollback ? ('. Will auto-revert to ' + (resp.rollbackSsid || 'previous network') + ' if it fails.') : '.'));
            return;
        }
        alert('Connect initiated to ' + ssid + '. Check status on the main page.');
    }).fail(function(){
        alert('Connect request failed');
    });
}

function initMqttDevicesUI() {
    var container = $('#mqttDevicesConfig');
    if (!container.length) return;

    function rowHtml(idx){
        var slotOptions = '';
        for (var s = 0; s <= 5; s++) {
            slotOptions += '<option value="' + s + '">' + s + '</option>';
        }
        return '' +
            '<div class="mqtt-dev-row" data-idx="' + idx + '">' +
                '<div class="mqtt-dev-grid">' +
                    '<div class="mqtt-col-name">' +
                        '<label class="mqtt-row-label">Name</label>' +
                        '<input type="text" class="ext-dev-name" maxlength="31" placeholder="Device ' + (idx + 1) + '">' +
                    '</div>' +
                    '<div class="mqtt-col-topic">' +
                        '<label class="mqtt-row-label">Topic</label>' +
                        '<input type="text" class="ext-dev-topic mqtt-topic-input" maxlength="95" placeholder="home/sensor/topic">' +
                    '</div>' +
                    '<div class="mqtt-col-slot">' +
                        '<label class="mqtt-row-label">Sensor Slot</label>' +
                        '<select class="ext-dev-slot">' + slotOptions + '</select>' +
                    '</div>' +
                    '<div class="mqtt-col-remove">' +
                        '<button type="button" class="btn ext-dev-remove mqtt-remove-btn">Remove</button>' +
                    '</div>' +
                '</div>' +
            '</div>';
    }

    container.empty();
    for (var i = 0; i < 4; i++) container.append(rowHtml(i));

    var visibleRows = 0;
    for (var j = 0; j < 4; j++) {
        var nm = configuration['extDev' + j + 'Name'] || '';
        var tp = configuration['extDev' + j + 'Topic'] || '';
        var sl = parseInt(configuration['extDev' + j + 'Slot'] || j, 10);
        var row = container.find('.mqtt-dev-row[data-idx="' + j + '"]');
        row.find('.ext-dev-name').val(nm);
        row.find('.ext-dev-topic').val(tp);
        row.find('.ext-dev-slot').val(String(isNaN(sl) ? j : sl));
        if (nm.length || tp.length) {
            row.show();
            visibleRows++;
        }
    }
    if (visibleRows === 0) {
        container.find('.mqtt-dev-row[data-idx="0"]').show();
        visibleRows = 1;
    }

    function updateAddButtonState(){
        var shown = container.find('.mqtt-dev-row:visible').length;
        $('#addMqttDeviceBtn').prop('disabled', shown >= 4).text(shown >= 4 ? 'Max 4 Devices' : 'Add Device');
    }
    updateAddButtonState();

    $('#addMqttDeviceBtn').off('click').on('click', function(){
        var next = container.find('.mqtt-dev-row:hidden').first();
        if (next.length) {
            next.show();
            updateAddButtonState();
        }
    });

    container.find('.ext-dev-remove').off('click').on('click', function(){
        var row = $(this).closest('.mqtt-dev-row');
        var idx = parseInt(row.attr('data-idx'), 10);
        row.find('.ext-dev-name').val('');
        row.find('.ext-dev-topic').val('');
        row.find('.ext-dev-slot').val('0');
        sendMsgToArduino('extDev' + idx + 'Name', '');
        sendMsgToArduino('extDev' + idx + 'Topic', '');
        sendMsgToArduino('extDev' + idx + 'Slot', '0');
        if (container.find('.mqtt-dev-row:visible').length > 1) row.hide();
        updateAddButtonState();
    });

    container.find('.ext-dev-name, .ext-dev-topic, .ext-dev-slot').off('change').on('change', function(){
        var row = $(this).closest('.mqtt-dev-row');
        var idx = parseInt(row.attr('data-idx'), 10);
        var name = row.find('.ext-dev-name').val() || '';
        var topic = row.find('.ext-dev-topic').val() || '';
        var slot = row.find('.ext-dev-slot').val() || '0';
        configuration['extDev' + idx + 'Name'] = name;
        configuration['extDev' + idx + 'Topic'] = topic;
        configuration['extDev' + idx + 'Slot'] = parseInt(slot,10) || 0;
        sendMsgToArduino('extDev' + idx + 'Name', name);
        sendMsgToArduino('extDev' + idx + 'Topic', topic);
        sendMsgToArduino('extDev' + idx + 'Slot', slot);
    });
}

function initFeatureModules(){
    initDebugToggle();
    initUICustomization();
    initSystemInfo();

    $('#wifi-scan-btn').on('click', function(){ scanWifi(); });
    initMqttDevicesUI();

    $('#wifi-connect-save-btn').on('click', function(){
        var ssid = $('#wifiSsid').val();
        var psw = $('#wifiPsw').val();
        if (!ssid) { alert('Enter SSID'); return; }

        var btn = $(this);
        btn.prop('disabled', true).text('Connecting...');
        apiPost('/connectWifi', { ssid: ssid, psw: psw, save: 'true' }).done(function(resp){
            if (typeof resp === 'string') {
                try { resp = JSON.parse(resp); } catch(e){ resp = {}; }
            }
            if (resp && resp.status === 'already_connected') {
                btn.prop('disabled', false).text('Connect & Save');
                alert('Already connected to ' + (resp.ssid || ssid) + (resp.ip ? (' (' + resp.ip + ')') : ''));
                return;
            }
            if (resp && resp.status === 'busy') {
                btn.prop('disabled', false).text('Connect & Save');
                alert('Another WiFi connect is already running. Please wait and try again.');
                return;
            }
            if (resp && resp.status === 'connecting' && resp.rollback) {
                console.log('WiFi switch started with rollback to', resp.rollbackSsid || 'previous network');
            }
            var attempts = 0;
            var timer = setInterval(function(){
                apiGet('/wifiStatus').done(function(st){
                    if (typeof st === 'string') { try { st = JSON.parse(st); } catch(e){ st = {}; } }
                    if (st.wifiSwitchResult === 'rollback_started') {
                        console.log('Rollback started to', st.wifiSwitchRollbackSsid || 'known network');
                    }
                    if (st.wifiSwitchResult === 'rollback_connected') {
                        clearInterval(timer);
                        btn.prop('disabled', false).text('Connect & Save');
                        alert('Connection to ' + ssid + ' failed. Restored previous network: ' + (st.ssid || st.wifiSwitchRollbackSsid || 'known network'));
                        return;
                    }
                    if (st.wifiSwitchResult === 'connect_failed' && !st.wifiSwitchPending) {
                        clearInterval(timer);
                        btn.prop('disabled', false).text('Connect & Save');
                        alert('Connection to ' + ssid + ' failed.');
                        return;
                    }
                    if (st.statusStr == 'Connected') {
                        var currentSsid = st.ssid || '';
                        if (currentSsid && currentSsid !== ssid) {
                            clearInterval(timer);
                            btn.prop('disabled', false).text('Connect & Save');
                            alert('Could not connect to ' + ssid + '. Stayed on ' + currentSsid + '.');
                            return;
                        }
                        clearInterval(timer);
                        btn.prop('disabled', false).text('Connect & Save');
                        alert('Connected: ' + (st.ip || ''));
                        return;
                    }
                });
                attempts++;
                if (attempts > 10) {
                    clearInterval(timer);
                    btn.prop('disabled', false).text('Connect & Save');
                    alert('Connection timeout');
                }
            }, 2000);
        }).fail(function(){
            alert('Connect request failed');
            $('#wifi-connect-save-btn').prop('disabled', false).text('Connect & Save');
        });
    });
}
