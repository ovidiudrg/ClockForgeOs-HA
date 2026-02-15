// -----------------------
// Debug toggle (UI only)
// -----------------------
var debugRefreshHandle = null;

function initDebugToggle(){
    var $btn = $('#debug-toggle-btn');
    if($btn.length === 0){ return; }

    var saved = null;
    try { saved = localStorage.getItem('clockforgeos_debug_enabled'); } catch(e) {}
    var enabled = (saved === '1');

    applyDebugUiState(enabled, false);

    $btn.off('click').on('click', function(){
        toggleDebug();
    });
}

function toggleDebug(){
    var enabled = $('body').hasClass('debug-enabled');
    enabled = !enabled;
    try { localStorage.setItem('clockforgeos_debug_enabled', enabled ? '1' : '0'); } catch(e) {}
    applyDebugUiState(enabled, true);
}

function applyDebugUiState(enabled, persistToFirmware){
    if (persistToFirmware === undefined) persistToFirmware = true;
    var $btn = $('#debug-toggle-btn');
    if(enabled){
        $('body').addClass('debug-enabled');
        $btn.addClass('debug-on').text('Disable Debug');

        if (typeof liveLogSetEnabled === 'function') {
            liveLogSetEnabled(true);
        }

        if (persistToFirmware) {
            postSetting('debugEnabled', true);
        }

        getClockDetails();

        if(debugRefreshHandle){ clearInterval(debugRefreshHandle); }
        debugRefreshHandle = setInterval(function(){
            if($('#clock-details').is(':visible')){
                getClockDetails();
            }
        }, 5000);
    } else {
        $('body').removeClass('debug-enabled');
        $btn.removeClass('debug-on').text('Enable Debug');

        if (typeof liveLogSetEnabled === 'function') {
            liveLogSetEnabled(false);
        }

        if (persistToFirmware) {
            postSetting('debugEnabled', false);
        }

        if(debugRefreshHandle){ clearInterval(debugRefreshHandle); debugRefreshHandle = null; }
        $('#clock-details').html('');
    }
}
