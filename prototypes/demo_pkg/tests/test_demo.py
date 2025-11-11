def test_package_imports():
    # simple import test to ensure package loads in test environment
    import importlib
    talker = importlib.import_module('demo_package.talker')
    listener = importlib.import_module('demo_package.listener')
    assert hasattr(talker, 'Talker')
    assert hasattr(listener, 'Listener')
