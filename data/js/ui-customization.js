// ============================================================================
// UI Customization: Page width, Background, Panel/Card colors
// ============================================================================
function applyUICSSVariables(uiWidth, uiBg, uiPanel, uiAccent, uiText, uiScale) {
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
    if (uiScale !== undefined && uiScale !== null && uiScale !== '') {
        var scaleNum = parseFloat(uiScale);
        if (!(scaleNum > 0)) scaleNum = 1;
        if (scaleNum < 0.7) scaleNum = 0.7;
        if (scaleNum > 1.2) scaleNum = 1.2;
        root.style.setProperty('--ui-scale', String(scaleNum));
    }
}

function applyUITheme(theme) {
    var t = String(theme || 'original').toLowerCase();
    if (t !== 'retro' && t !== 'vfd') t = 'original';
    var root = document.documentElement;
    var retroLink = document.getElementById('retroThemeStylesheet');
    var vfdLink = document.getElementById('vfdThemeStylesheet');
    if (retroLink) {
        retroLink.disabled = (t !== 'retro');
    }
    if (vfdLink) {
        vfdLink.disabled = (t !== 'vfd');
    }
    root.setAttribute('data-ui-theme', t);
}

function normalizeThemeValue(theme) {
    var t = String(theme || '').toLowerCase();
    if (t === 'original' || t === 'retro' || t === 'vfd') return t;
    return '';
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

function getAutoTextForFirmware(conf) {
    // Nixie: warm text; VFD: cool/cyan text
    return isVfdFirmwareProfile(conf) ? '#c7f6ff' : '#e7c39f';
}

function isAutoDefaultText(value) {
    var c = normalizeHexColor(value);
    return c === '#dddddd' || c === '#c7f6ff' || c === '#e7c39f';
}

function getDefaultThemeForFirmware(conf) {
    var fwTheme = normalizeThemeValue(conf && conf.uiDefaultTheme);
    if (fwTheme === 'retro' || fwTheme === 'vfd') {
        return fwTheme;
    }
    return isVfdFirmwareProfile(conf) ? 'vfd' : 'retro';
}

function isAutoThemeEnabled() {
    var raw = localStorage.getItem('ui_auto_theme');
    if (raw === null || raw === undefined || raw === '') return true;
    raw = String(raw).toLowerCase();
    return !(raw === '0' || raw === 'false' || raw === 'off');
}

function loadUICSSFromLocalStorage() {
    try {
        var profile = isVfdFirmwareProfile(configuration) ? 'vfd' : 'nixie';
        document.documentElement.setAttribute('data-display-profile', profile);

        var autoTheme = isAutoThemeEnabled();
        localStorage.setItem('ui_auto_theme', autoTheme ? '1' : '0');

        var firmwareTheme = getDefaultThemeForFirmware(configuration);
        var theme = '';
        if (autoTheme) {
            theme = firmwareTheme;
            localStorage.setItem('ui_theme_auto_last', theme);
        } else {
            theme = normalizeThemeValue(localStorage.getItem('ui_theme'));
            if (!theme) {
                theme = 'original';
            }
        }
        applyUITheme(theme);

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
        
        var autoText = getAutoTextForFirmware(configuration);
        var storedText = localStorage.getItem('ui_text');
        var configText = (configuration && configuration.uiTextColor) ? configuration.uiTextColor : '';
        var hasCustomStoredText = !!normalizeHexColor(storedText) && !isAutoDefaultText(storedText);
        var hasCustomConfigText = !!normalizeHexColor(configText) && !isAutoDefaultText(configText);

        var uiText = autoText;
        if (hasCustomConfigText) {
            uiText = configText;
        }
        if (hasCustomStoredText) {
            uiText = storedText;
        }

        var uiScale = 1;
        var lsScale = parseFloat(localStorage.getItem('ui_scale') || '1');
        if (lsScale > 0) uiScale = lsScale;
        if (window.innerWidth && window.innerWidth <= 768) {
            uiScale = 1;
        }
        
        applyUICSSVariables(uiWidth, uiBg, uiPanel, uiAccent, uiText, uiScale);
        
        if ($('#uiWidth').length) {
            var widthVal = (configuration && configuration.uiWidth) ? configuration.uiWidth : (uiWidth === '100%' ? '100%' : parseInt(uiWidth));
            $('#uiWidth').val(widthVal);
        }
        if ($('#uiBgColor').length) $('#uiBgColor').val(uiBg);
        if ($('#uiPanelColor').length) $('#uiPanelColor').val(uiPanel);
        if ($('#uiAccentColor').length) $('#uiAccentColor').val(uiAccent);
        if ($('#uiTextColor').length) $('#uiTextColor').val(uiText);
        if ($('#uiScale').length) {
            var uiScalePercent = Math.round(uiScale * 100);
            if (uiScalePercent < 70) uiScalePercent = 70;
            if (uiScalePercent > 120) uiScalePercent = 120;
            var uiScalePercentStr = String(uiScalePercent);
            if ($('#uiScale option[value="' + uiScalePercentStr + '"]').length === 0) {
                uiScalePercentStr = '100';
            }
            $('#uiScale').val(uiScalePercentStr);
        }
        if ($('#uiTheme').length) {
            if (theme === 'retro' || theme === 'vfd') {
                $('#uiTheme').val(theme);
            } else {
                $('#uiTheme').val('original');
            }
            $('#uiTheme').prop('disabled', autoTheme);
        }
        if ($('#uiAutoTheme').length) {
            $('#uiAutoTheme').prop('checked', autoTheme);
        }
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

    $('#uiAutoTheme').on('change', function(){
        var enabled = !!$(this).is(':checked');
        localStorage.setItem('ui_auto_theme', enabled ? '1' : '0');
        if (enabled) {
            var autoTheme = getDefaultThemeForFirmware(configuration);
            applyUITheme(autoTheme);
            localStorage.setItem('ui_theme_auto_last', autoTheme);
            if ($('#uiTheme').length) {
                $('#uiTheme').val(autoTheme);
                $('#uiTheme').prop('disabled', true);
            }
            return;
        }

        if ($('#uiTheme').length) {
            $('#uiTheme').prop('disabled', false);
        }
        var manualTheme = normalizeThemeValue(localStorage.getItem('ui_theme'));
        if (!manualTheme) manualTheme = 'original';
        applyUITheme(manualTheme);
        if ($('#uiTheme').length) {
            $('#uiTheme').val(manualTheme);
        }
    });

    $('#uiTheme').on('change', function(){
        var selected = String($(this).val() || '').toLowerCase();
        var val = (selected === 'retro' || selected === 'vfd') ? selected : 'original';
        if (isAutoThemeEnabled()) {
            var autoTheme = getDefaultThemeForFirmware(configuration);
            applyUITheme(autoTheme);
            $(this).val(autoTheme);
            return;
        }
        applyUITheme(val);
        saveUICSSToLocalStorage('theme', val);
    });
    
    $('#uiWidth').on('change', function(){
        var val = $(this).val();
        var cssVal = (val === '100%') ? '100%' : val + 'px';
        applyUICSSVariables(cssVal, null, null, null, null, null);
        saveUICSSToLocalStorage('width', cssVal);
        sendMsgToArduino('uiWidth', val);
    });
    
    $('#uiBgColor').on('input change', function(event){
        var val = $(this).val();
        applyUICSSVariables(null, val, null, null, null, null);
        saveUICSSToLocalStorage('bg', val);
        if(event.type === 'change') {
            sendMsgToArduino('uiBgColor', val);
        }
    });
    
    $('#uiPanelColor').on('input change', function(event){
        var val = $(this).val();
        applyUICSSVariables(null, null, val, null, null, null);
        saveUICSSToLocalStorage('panel', val);
        if(event.type === 'change') {
            sendMsgToArduino('uiPanelColor', val);
        }
    });
    
    $('#uiAccentColor').on('input change', function(event){
        var val = $(this).val();
        applyUICSSVariables(null, null, null, val, null, null);
        saveUICSSToLocalStorage('accent', val);
        if(event.type === 'change') {
            sendMsgToArduino('uiAccentColor', val);
        }
    });
    
    $('#uiTextColor').on('input change', function(event){
        var val = $(this).val();
        applyUICSSVariables(null, null, null, null, val, null);
        saveUICSSToLocalStorage('text', val);
        if(event.type === 'change') {
            sendMsgToArduino('uiTextColor', val);
        }
    });

    $('#uiScale').on('change', function(){
        var val = parseInt($(this).val(), 10);
        if (Number.isNaN(val)) val = 100;
        if (val < 70) val = 70;
        if (val > 120) val = 120;
        var cssScale = val / 100;
        applyUICSSVariables(null, null, null, null, null, cssScale);
        saveUICSSToLocalStorage('scale', String(cssScale));
    });
}
