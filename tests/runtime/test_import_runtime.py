def test_import_runtime_package() -> None:
    import PhyAgentOS.runtime

    assert PhyAgentOS.runtime.__version__
