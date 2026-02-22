import 'package:flutter/material.dart';

import '../services/clockforge_api.dart';

class RgbScreen extends StatefulWidget {
  const RgbScreen({
    super.key,
    required this.api,
  });

  final ClockForgeApi api;

  @override
  State<RgbScreen> createState() => _RgbScreenState();
}

class _RgbScreenState extends State<RgbScreen> {
  bool _loading = true;
  String? _error;
  int _lastReloginEventSeq = 0;

  int _rgbEffect = 1;
  double _rgbBrightness = 100;
  double _rgbSpeed = 50;
  bool _rgbNightEnabled = false;
  double _maxLedmA = 350;

  double _r = 255;
  double _g = 255;
  double _b = 255;

  static const List<DropdownMenuItem<int>> _effectItems = [
    DropdownMenuItem(value: 0, child: Text('0 - Off')),
    DropdownMenuItem(value: 1, child: Text('1 - Fixed Color')),
    DropdownMenuItem(value: 2, child: Text('2')),
    DropdownMenuItem(value: 3, child: Text('3')),
    DropdownMenuItem(value: 4, child: Text('4')),
    DropdownMenuItem(value: 5, child: Text('5')),
    DropdownMenuItem(value: 6, child: Text('6')),
    DropdownMenuItem(value: 7, child: Text('7')),
    DropdownMenuItem(value: 8, child: Text('8')),
    DropdownMenuItem(value: 9, child: Text('9')),
    DropdownMenuItem(value: 10, child: Text('10')),
    DropdownMenuItem(value: 11, child: Text('11')),
  ];

  @override
  void initState() {
    super.initState();
    widget.api.reloginEvent.addListener(_onReloginEvent);
    _load();
  }

  @override
  void dispose() {
    widget.api.reloginEvent.removeListener(_onReloginEvent);
    super.dispose();
  }

  void _onReloginEvent() {
    if (!mounted) return;
    final event = widget.api.reloginEvent.value;
    if (event == null) return;
    if (event.seq <= _lastReloginEventSeq) return;
    _lastReloginEventSeq = event.seq;

    final messenger = ScaffoldMessenger.of(context);
    messenger.hideCurrentSnackBar();
    messenger.showSnackBar(
      SnackBar(
        content: Text(event.message),
        backgroundColor: event.success
            ? Colors.green.shade700
            : Theme.of(context).colorScheme.error,
      ),
    );
  }

  int _asInt(dynamic value, int fallback) {
    if (value is num) return value.toInt();
    return int.tryParse(value?.toString() ?? '') ?? fallback;
  }

  bool _asBool(dynamic value, {bool fallback = false}) {
    if (value is bool) return value;
    if (value is num) return value != 0;
    final v = (value?.toString() ?? '').toLowerCase().trim();
    if (v == 'true' || v == '1' || v == 'yes' || v == 'on') return true;
    if (v == 'false' || v == '0' || v == 'no' || v == 'off') return false;
    return fallback;
  }

  Future<void> _load() async {
    setState(() {
      _loading = true;
      _error = null;
    });

    try {
      final cfg = await widget.api.getConfiguration();
      setState(() {
        _rgbEffect = _asInt(cfg['rgbEffect'], 1).clamp(0, 11);
        _rgbBrightness = _asInt(cfg['rgbBrightness'], 100).toDouble().clamp(0, 255);
        _rgbSpeed = _asInt(cfg['rgbSpeed'], 50).toDouble().clamp(1, 255);
        _rgbNightEnabled = _asBool(cfg['rgbNightEnabled']);
        _maxLedmA = _asInt(cfg['maxLedmA'], 350).toDouble().clamp(0, 2000);

        _r = _asInt(cfg['rgbFixR'], 255).toDouble().clamp(0, 255);
        _g = _asInt(cfg['rgbFixG'], 255).toDouble().clamp(0, 255);
        _b = _asInt(cfg['rgbFixB'], 255).toDouble().clamp(0, 255);
      });
    } catch (e) {
      _error = e.toString();
    } finally {
      if (mounted) setState(() => _loading = false);
    }
  }

  Future<void> _saveValue(String key, String value) async {
    try {
      await widget.api.saveSetting(key: key, value: value);
      if (!mounted) return;
      setState(() => _error = null);
    } catch (e) {
      if (!mounted) return;
      setState(() => _error = e.toString());
    }
  }

  Widget _colorPreview() {
    return Container(
      height: 56,
      decoration: BoxDecoration(
        color: Color.fromARGB(255, _r.toInt(), _g.toInt(), _b.toInt()),
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: Colors.black26),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('RGB'),
        actions: [
          IconButton(
            tooltip: 'Reload',
            onPressed: _loading ? null : _load,
            icon: const Icon(Icons.refresh),
          ),
        ],
      ),
      body: _loading
          ? const Center(child: CircularProgressIndicator())
          : ListView(
              padding: const EdgeInsets.all(16),
              children: [
                ValueListenableBuilder<bool>(
                  valueListenable: widget.api.isReloginInProgress,
                  builder: (context, inProgress, _) {
                    if (!inProgress) return const SizedBox.shrink();
                    return Card(
                      child: Padding(
                        padding: const EdgeInsets.all(12),
                        child: Row(
                          children: const [
                            SizedBox(
                              width: 18,
                              height: 18,
                              child: CircularProgressIndicator(strokeWidth: 2),
                            ),
                            SizedBox(width: 12),
                            Expanded(child: Text('Session expired. Re-authenticating...')),
                          ],
                        ),
                      ),
                    );
                  },
                ),
                ValueListenableBuilder<bool>(
                  valueListenable: widget.api.isReloginInProgress,
                  builder: (context, inProgress, _) =>
                      inProgress ? const SizedBox(height: 12) : const SizedBox.shrink(),
                ),
                if (_error != null)
                  Card(
                    color: Theme.of(context).colorScheme.errorContainer,
                    child: Padding(
                      padding: const EdgeInsets.all(12),
                      child: Text(_error!),
                    ),
                  ),
                Card(
                  child: Padding(
                    padding: const EdgeInsets.all(12),
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        const Text('Effect'),
                        DropdownButton<int>(
                          value: _rgbEffect,
                          isExpanded: true,
                          items: _effectItems,
                          onChanged: (v) {
                            if (v == null) return;
                            setState(() => _rgbEffect = v);
                            _saveValue('rgbEffect', '$v');
                          },
                        ),
                        const SizedBox(height: 8),
                        Text('Brightness: ${_rgbBrightness.toInt()}'),
                        Slider(
                          value: _rgbBrightness,
                          min: 0,
                          max: 255,
                          onChanged: (v) => setState(() => _rgbBrightness = v),
                          onChangeEnd: (v) => _saveValue('rgbBrightness', '${v.toInt()}'),
                        ),
                        Text('Speed: ${_rgbSpeed.toInt()}'),
                        Slider(
                          value: _rgbSpeed,
                          min: 1,
                          max: 255,
                          onChanged: (v) => setState(() => _rgbSpeed = v),
                          onChangeEnd: (v) => _saveValue('rgbSpeed', '${v.toInt()}'),
                        ),
                        SwitchListTile(
                          contentPadding: EdgeInsets.zero,
                          title: const Text('RGB Night Enabled'),
                          value: _rgbNightEnabled,
                          onChanged: (v) {
                            setState(() => _rgbNightEnabled = v);
                            _saveValue('rgbNightEnabled', v ? 'true' : 'false');
                          },
                        ),
                        Text('LED Current Limit: ${_maxLedmA.toInt()} mA'),
                        Slider(
                          value: _maxLedmA,
                          min: 0,
                          max: 2000,
                          divisions: 40,
                          onChanged: (v) => setState(() => _maxLedmA = v),
                          onChangeEnd: (v) => _saveValue('maxLedmA', '${v.toInt()}'),
                        ),
                      ],
                    ),
                  ),
                ),
                Card(
                  child: Padding(
                    padding: const EdgeInsets.all(12),
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        const Text('Fixed Color (RGB)'),
                        const SizedBox(height: 8),
                        _colorPreview(),
                        const SizedBox(height: 10),
                        Text('R: ${_r.toInt()}'),
                        Slider(
                          value: _r,
                          min: 0,
                          max: 255,
                          activeColor: Colors.red,
                          onChanged: (v) => setState(() => _r = v),
                          onChangeEnd: (v) => _saveValue('rgbFixR', '${v.toInt()}'),
                        ),
                        Text('G: ${_g.toInt()}'),
                        Slider(
                          value: _g,
                          min: 0,
                          max: 255,
                          activeColor: Colors.green,
                          onChanged: (v) => setState(() => _g = v),
                          onChangeEnd: (v) => _saveValue('rgbFixG', '${v.toInt()}'),
                        ),
                        Text('B: ${_b.toInt()}'),
                        Slider(
                          value: _b,
                          min: 0,
                          max: 255,
                          activeColor: Colors.blue,
                          onChanged: (v) => setState(() => _b = v),
                          onChangeEnd: (v) => _saveValue('rgbFixB', '${v.toInt()}'),
                        ),
                      ],
                    ),
                  ),
                ),
              ],
            ),
    );
  }
}
