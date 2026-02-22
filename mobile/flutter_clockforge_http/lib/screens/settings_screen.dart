import 'package:flutter/material.dart';

import '../services/clockforge_api.dart';

class SettingsScreen extends StatefulWidget {
  const SettingsScreen({
    super.key,
    required this.api,
  });

  final ClockForgeApi api;

  @override
  State<SettingsScreen> createState() => _SettingsScreenState();
}

class _SettingsScreenState extends State<SettingsScreen> {
  bool _loading = true;
  String? _error;
  int _lastReloginEventSeq = 0;

  bool _displayPower = true;
  bool _enableTimeDisplay = true;
  bool _enableTempDisplay = true;
  bool _enableHumidDisplay = true;
  bool _enablePressDisplay = true;
  bool _wakeOnMotionEnabled = true;

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

  bool _asBool(dynamic value, {bool fallback = false}) {
    if (value is bool) return value;
    if (value is num) return value != 0;
    if (value is String) {
      final v = value.toLowerCase().trim();
      return v == 'true' || v == '1' || v == 'on' || v == 'yes';
    }
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
        _displayPower = _asBool(cfg['displayPower'], fallback: true);
        _enableTimeDisplay = _asBool(cfg['enableTimeDisplay'], fallback: true);
        _enableTempDisplay = _asBool(cfg['enableTempDisplay'], fallback: true);
        _enableHumidDisplay = _asBool(cfg['enableHumidDisplay'], fallback: true);
        _enablePressDisplay = _asBool(cfg['enablePressDisplay'], fallback: true);
        _wakeOnMotionEnabled = _asBool(cfg['wakeOnMotionEnabled'], fallback: true);
      });
    } catch (e) {
      _error = e.toString();
    } finally {
      if (mounted) {
        setState(() => _loading = false);
      }
    }
  }

  Future<void> _saveBool(String key, bool value) async {
    try {
      await widget.api.saveSetting(key: key, value: value ? 'true' : 'false');
      if (!mounted) return;
      setState(() => _error = null);
    } catch (e) {
      if (!mounted) return;
      setState(() => _error = e.toString());
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Settings'),
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
                SwitchListTile(
                  title: const Text('Display Power'),
                  value: _displayPower,
                  onChanged: (v) {
                    setState(() => _displayPower = v);
                    _saveBool('displayPower', v);
                  },
                ),
                SwitchListTile(
                  title: const Text('Show Time'),
                  value: _enableTimeDisplay,
                  onChanged: (v) {
                    setState(() => _enableTimeDisplay = v);
                    _saveBool('enableTimeDisplay', v);
                  },
                ),
                SwitchListTile(
                  title: const Text('Show Temperature'),
                  value: _enableTempDisplay,
                  onChanged: (v) {
                    setState(() => _enableTempDisplay = v);
                    _saveBool('enableTempDisplay', v);
                  },
                ),
                SwitchListTile(
                  title: const Text('Show Humidity'),
                  value: _enableHumidDisplay,
                  onChanged: (v) {
                    setState(() => _enableHumidDisplay = v);
                    _saveBool('enableHumidDisplay', v);
                  },
                ),
                SwitchListTile(
                  title: const Text('Show Pressure'),
                  value: _enablePressDisplay,
                  onChanged: (v) {
                    setState(() => _enablePressDisplay = v);
                    _saveBool('enablePressDisplay', v);
                  },
                ),
                SwitchListTile(
                  title: const Text('Wake On Motion'),
                  value: _wakeOnMotionEnabled,
                  onChanged: (v) {
                    setState(() => _wakeOnMotionEnabled = v);
                    _saveBool('wakeOnMotionEnabled', v);
                  },
                ),
              ],
            ),
    );
  }
}
