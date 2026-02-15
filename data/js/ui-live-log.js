/* ===== Live Log via SSE ===== */
var _evtSrc = null;
var _logMaxChars = 20000;
var _logTrimToChars = 16000;

function _logAppend(line){
  var el = $("#live-log");
  if (!el.length) return;
  var atBottom = (el[0].scrollTop + el[0].clientHeight + 20) >= el[0].scrollHeight;
  var nextText = el.text() + line + "\n";
  if (nextText.length > _logMaxChars) {
    nextText = nextText.slice(nextText.length - _logTrimToChars);
  }
  el.text(nextText);
  if (atBottom) el[0].scrollTop = el[0].scrollHeight;
}

function _startEventSource(){
  if (!$('body').hasClass('debug-enabled')) {
    $("#log-status").text("Disabled");
    return;
  }
  if (_evtSrc) return;
  try {
    _evtSrc = new EventSource("/events");
    _evtSrc.addEventListener("status", function(){ $("#log-status").text("Connected"); });
    _evtSrc.addEventListener("log", function(e){ _logAppend(e.data); });
    _evtSrc.onerror = function(){
      $("#log-status").text("Disconnected");
      try { _evtSrc.close(); } catch(e){}
      _evtSrc = null;
      if ($('body').hasClass('debug-enabled')) {
        setTimeout(_startEventSource, 2000);
      }
    };
  } catch(e) {
    $("#log-status").text("Disconnected");
    _evtSrc = null;
  }
}

function liveLogSetEnabled(enabled){
  if (enabled) {
    _startEventSource();
  } else {
    if (_evtSrc) {
      try { _evtSrc.close(); } catch(e){}
      _evtSrc = null;
    }
    $("#log-status").text("Disabled");
  }
}

function initLiveLog(){
  $("#log-clear-btn").on("click", function(){ $("#live-log").text(""); });
  liveLogSetEnabled($('body').hasClass('debug-enabled'));
}
