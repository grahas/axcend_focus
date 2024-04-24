# test_legacy_compatability_interface.py
import pytest
from axcend_focus_legacy_compatibility_layer.legacy_compatibility_interface import app

@pytest.fixture
def client():
    with app.test_client() as client:
        yield client

def test_home(client):
    response = client.get('/')
    assert response.status_code == 200
    assert response.data == b'Hello, World!1s'

