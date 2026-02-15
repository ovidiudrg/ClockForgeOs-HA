// ============================================================================
// UI Customization: Page width, Background, Panel/Card colors
// ============================================================================
function applyUICSSVariables(uiWidth, uiBg, uiPanel, uiAccent, uiText) {
    var root = document.documentElement;
    if (uiWidth) {
        root.style.setProperty('--ui-width', uiWidth.toString());
    }
    if (uiBg) {
        root.style.setProperty('--ui-bg', uiBg);
    }
    if (uiPanel) {
        root.style.setProperty('--ui-panel', uiPanel);
    }
    if (uiAccent) {
        root.style.setProperty('--ui-accent', uiAccent);
    }
    if (uiText) {
        root.style.setProperty('--ui-text', uiText);
    }
}

function normalizeHexColor(value) {
    if (!value) return '';
    return String(value).trim().toLowerCase();
}

function isVfdFirmwareProfile(conf) {
    var fw = normalizeHexColor(conf && conf.FW);
    var tubeDriver = normalizeHexColor(conf && conf.tubeDriver);
    var fingerprint = fw + ' ' + tubeDriver;

    // VFD-related fingerprints seen in this project
    return /vfd|iv-?11|iv-?18|max6921|pt6311|pt6355|sn75512|sn75518|samsung/.test(fingerprint);
}

function getAutoAccentForFirmware(conf) {
    // Nixie-like default orange (e.g., Z574M)
    var nixieAccent = '#d96025';
    // VFD-like cyan/blue-green glow
    var vfdAccent = '#66e8ff';
    return isVfdFirmwareProfile(conf) ? vfdAccent : nixieAccent;
}

function isAutoDefaultAccent(value) {
    var c = normalizeHexColor(value);
    return c === '#d96025' || c === '#66e8ff';
}

function loadUICSSFromLocalStorage() {
    try {
        var uiWidth = (configuration && configuration.uiWidth) ? 
            (configuration.uiWidth === '100%' || configuration.uiWidth === 0xFFFF ? '100%' : configuration.uiWidth + 'px') :
            (localStorage.getItem('ui_width') || '100%');
        if (window.innerWidth && window.innerWidth <= 768) {
            uiWidth = '100%';
        }
        
        var uiBg = (configuration && configuration.uiBgColor) ? 
            configuration.uiBgColor :
            (localStorage.getItem('ui_bg') || '#111111');
        
        var uiPanel = (configuration && configuration.uiPanelColor) ? 
            configuration.uiPanelColor :
            (localStorage.getItem('ui_panel') || '#161616');
        
        var autoAccent = getAutoAccentForFirmware(configuration);
        var storedAccent = localStorage.getItem('ui_accent');
        var configAccent = (configuration && configuration.uiAccentColor) ? configuration.uiAccentColor : '';
        var hasCustomStoredAccent = !!normalizeHexColor(storedAccent) && !isAutoDefaultAccent(storedAccent);
        var hasCustomConfigAccent = !!normalizeHexColor(configAccent) && !isAutoDefaultAccent(configAccent);

        var uiAccent = autoAccent;
        if (hasCustomConfigAccent) {
            uiAccent = configAccent;
        }
        if (hasCustomStoredAccent) {
            uiAccent = storedAccent;
        }
        
        var uiText = (configuration && configuration.uiTextColor) ? 
            configuration.uiTextColor :
            (localStorage.getItem('ui_text') || '#DDDDDD');
        
        applyUICSSVariables(uiWidth, uiBg, uiPanel, uiAccent, uiText);
        
        if ($('#uiWidth').length) {
            var widthVal = (configuration && configuration.uiWidth) ? configuration.uiWidth : (uiWidth === '100%' ? '100%' : parseInt(uiWidth));
            $('#uiWidth').val(widthVal);
        }
        if ($('#uiBgColor').length) $('#uiBgColor').val(uiBg);
        if ($('#uiPanelColor').length) $('#uiPanelColor').val(uiPanel);
        if ($('#uiAccentColor').length) $('#uiAccentColor').val(uiAccent);
        if ($('#uiTextColor').length) $('#uiTextColor').val(uiText);
    } catch(e) {
        console.warn('Failed to load UI CSS from settings:', e);
    }
}

function saveUICSSToLocalStorage(key, value) {
    try {
        localStorage.setItem('ui_' + key, value);
    } catch(e) {
        console.warn('Failed to save UI CSS to localStorage:', e);
    }
}

function initUICustomization() {
    loadUICSSFromLocalStorage();
    
    $('#uiWidth').on('change', function(){
        var val = $(this).val();
        var cssVal = (val === '100%') ? '100%' : val + 'px';
        applyUICSSVariables(cssVal, null, null, null, null);
        saveUICSSToLocalStorage('width', cssVal);
        sendMsgToArduino('uiWidth', val);
    });
    
    $('#uiBgColor').on('input change', function(event){
        var val = $(this).val();
        applyUICSSVariables(null, val, null, null, null);
        saveUICSSToLocalStorage('bg', val);
        if(event.type === 'change') {
            sendMsgToArduino('uiBgColor', val);
        }
    });
    
    $('#uiPanelColor').on('input change', function(event){
        var val = $(this).val();
        applyUICSSVariables(null, null, val, null, null);
        saveUICSSToLocalStorage('panel', val);
        if(event.type === 'change') {
            sendMsgToArduino('uiPanelColor', val);
        }
    });
    
    $('#uiAccentColor').on('input change', function(event){
        var val = $(this).val();
        applyUICSSVariables(null, null, null, val, null);
        saveUICSSToLocalStorage('accent', val);
        if(event.type === 'change') {
            sendMsgToArduino('uiAccentColor', val);
        }
    });
    
    $('#uiTextColor').on('input change', function(event){
        var val = $(this).val();
        applyUICSSVariables(null, null, null, null, val);
        saveUICSSToLocalStorage('text', val);
        if(event.type === 'change') {
            sendMsgToArduino('uiTextColor', val);
        }
    });
}
