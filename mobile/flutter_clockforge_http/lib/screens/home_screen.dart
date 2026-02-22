import 'dart:async';

import 'package:flutter/material.dart';

import '../models/current_infos.dart';
import 'rgb_screen.dart';
import 'settings_screen.dart';
import '../services/clockforge_api.dart';

class HomeScreen extends StatefulWidget {
  const HomeScreen({super.key});

  @override
  State<HomeScreen> createState() => _HomeScreenState();
}

class _HomeScreenState extends State<HomeScreen> {
  final _baseUrlCtrl = TextEditingController(text: 'http://192.168.1.120');
  final _passwordCtrl = TextEditingController();
  ClockForgeApi? _api;
  ClockForgeApi? _apiWithReloginListener;
  Timer? _pollTimer;
  int _lastReloginEventSeq = 0;

  CurrentInfos? _status;
  String? _error;
  bool _loading = false;
  bool _displayPower = true;

  Future<void> _openSettings() async {
    final api = _api;
    if (api == null) return;
    await Navigator.of(context).push(
      MaterialPageRoute(
        builder: (_) => SettingsScreen(api: api),
      ),
    );
  }

  Future<void> _openRgb() async {
    final api = _api;
    if (api == null) return;
    await Navigator.of(context).push(
      MaterialPageRoute(
        builder: (_) => RgbScreen(api: api),
      ),
    );
  }

  @override
  void dispose() {
    _pollTimer?.cancel();
    _bindReloginListener(null);
    _api?.dispose();
    _baseUrlCtrl.dispose();
    _passwordCtrl.dispose();
    super.dispose();
  }

  void _bindReloginListener(ClockForgeApi? api) {
    final oldApi = _apiWithReloginListener;
    if (oldApi != null) {
      oldApi.reloginEvent.removeListener(_onReloginEvent);
    }
    _apiWithReloginListener = api;
    _lastReloginEventSeq = 0;
    if (api != null) {
      api.reloginEvent.addListener(_onReloginEvent);
    }
  }

  void _onReloginEvent() {
    if (!mounted) return;
    final event = _apiWithReloginListener?.reloginEvent.value;
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

  Future<void> _connectAndLogin() async {
    setState(() {
      _loading = true;
      _error = null;
    });

    _pollTimer?.cancel();
    _bindReloginListener(null);
    _api?.dispose();
    final api = ClockForgeApi(baseUrl: _baseUrlCtrl.text.trim());

    try {
      await api.login(_passwordCtrl.text);
      _api = api;
      _bindReloginListener(api);
      await _refresh();
      _pollTimer = Timer.periodic(const Duration(milliseconds: 1500), (_) => _refresh());
    } catch (e) {
      api.dispose();
      _error = e.toString();
    } finally {
      if (mounted) {
        setState(() => _loading = false);
      }
    }
  }

  Future<void> _refresh() async {
    final api = _api;
    if (api == null) return;

    try {
      final data = await api.getCurrentInfos();
      if (!mounted) return;
      setState(() {
        _status = data;
        _error = null;
      });
    } catch (e) {
      if (!mounted) return;
      setState(() => _error = e.toString());
    }
  }

  Future<void> _setDisplayPower(bool enabled) async {
    final api = _api;
    if (api == null) return;

    setState(() {
      _displayPower = enabled;
      _error = null;
    });

    try {
      await api.saveSetting(
        key: 'displayPower',
        value: enabled ? 'true' : 'false',
      );
    } catch (e) {
      if (!mounted) return;
      setState(() => _error = e.toString());
    }
  }

  String _fmtDouble(double? value, {String suffix = ''}) {
    if (value == null) return '--';
    return '${value.toStringAsFixed(1)}$suffix';
  }

  @override
  Widget build(BuildContext context) {
    final api = _api;
    final status = _status;

    return Scaffold(
      appBar: AppBar(
        title: const Text('ClockForge Mobile (HTTP)'),
        actions: [
          IconButton(
            tooltip: 'RGB',
            onPressed: _api == null ? null : _openRgb,
            icon: const Icon(Icons.palette),
          ),
          IconButton(
            tooltip: 'Settings',
            onPressed: _api == null ? null : _openSettings,
            icon: const Icon(Icons.settings),
          ),
        ],
      ),
      body: ListView(
        padding: const EdgeInsets.all(16),
        children: [
          if (api != null)
            ValueListenableBuilder<bool>(
              valueListenable: api.isReloginInProgress,
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
          if (api != null) const SizedBox(height: 12),
          TextField(
            controller: _baseUrlCtrl,
            decoration: const InputDecoration(
              labelText: 'Clock Base URL',
              hintText: 'http://192.168.1.120',
            ),
          ),
          const SizedBox(height: 12),
          TextField(
            controller: _passwordCtrl,
            decoration: const InputDecoration(labelText: 'Admin Password'),
            obscureText: true,
          ),
          const SizedBox(height: 12),
          FilledButton(
            onPressed: _loading ? null : _connectAndLogin,
            child: Text(_loading ? 'Connecting...' : 'Connect + Login'),
          ),
          const SizedBox(height: 20),
          if (_error != null)
            Card(
              color: Theme.of(context).colorScheme.errorContainer,
              child: Padding(
                padding: const EdgeInsets.all(12),
                child: Text(_error!),
              ),
            ),
          if (status != null) ...[
            Card(
              child: Padding(
                padding: const EdgeInsets.all(12),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text('Time: ${status.currentDate} ${status.currentTime}'),
                    Text('Source: ${status.timeSource}'),
                    Text('RSSI: ${status.rssi ?? '--'} dBm'),
                    Text('Mode: ${status.dayNightMode}'),
                    Text('Tubes: ${status.tubesPower ? 'ON' : 'OFF'}'),
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
                    Text('Temperature: ${_fmtDouble(status.temperature1, suffix: ' C')}'),
                    Text('Humidity: ${_fmtDouble(status.humidity1, suffix: ' %')}'),
                    Text('Pressure: ${_fmtDouble(status.pressure, suffix: ' hPa')}'),
                  ],
                ),
              ),
            ),
            SwitchListTile(
              title: const Text('Display Power'),
              subtitle: const Text('Writes /saveSetting key=displayPower'),
              value: _displayPower,
              onChanged: _setDisplayPower,
            ),
          ],
        ],
      ),
    );
  }
}
