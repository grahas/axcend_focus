# test_legacy_compatability_interface.py
import pytest
from axcend_focus_legacy_compatibility_layer.legacy_compatibility_interface import app, system_state


@pytest.fixture
def client():
    with app.test_client() as client:
        yield client

@pytest.fixture(autouse=True)
def reset_system_state():
    """Reset the system state before and after each test."""
    # Code to reset the system state goes here
    system_state["is_reserved"] = False
    system_state["operator"] = None
    system_state["operator_ip"] = None

    # Yield control to the test function
    yield

    # Code to clean up after the test goes here (if necessary)


def test_home(client):
    """Test the home route."""
    response = client.get("/")
    assert response.status_code == 200
    assert response.data == b"Hello, World!"


def test_is_reserved(client):
    """Test the is_reserved route."""
    response = client.get("/IsReserved")
    assert response.status_code == 200
    assert b"NOT RESERVED" in response.data


def test_reserve_and_release_system(client):
    """Test the reserve_system and release_system routes together."""
    # Test reserve_system route
    response = client.get("/ReserveSystem?user=test_user")
    assert response.status_code == 200
    assert b"RESERVED" in response.data
    test_key = response.json["result"]["key"]

    # Test release_system route
    response = client.get(f"/ReleaseSystem?key={test_key}")
    assert response.status_code == 200
    assert b"OK" in response.data


def test_reserve_system_already_reserved(client):
    """Test the reserve_system route when the system is already reserved."""
    client.get("/ReserveSystem?user=test_user")
    response = client.get("/ReserveSystem?user=test_user")
    assert response.status_code == 200
    assert b"ALREADY RESERVED" in response.data


def test_release_system_invalid_key(client):
    """Test the release_system route with an invalid key."""
    client.get("/ReserveSystem?user=test_user")
    response = client.get("/ReleaseSystem?key=invalid_key")
    assert response.status_code == 200
    assert b"INVALID KEY" in response.data


def test_release_system_not_reserved(client):
    """Test the release_system route when the system is not reserved."""
    response = client.get("/ReleaseSystem?key=test_key")
    assert response.status_code == 200
    assert b"NOT RESERVED" in response.data


def test_get_machine_name(client):
    """Test the machinename route."""
    response = client.get("/machinename")
    assert response.status_code == 200


def test_get_version(client):
    """Test the version route."""
    response = client.get("/version")
    assert response.status_code == 200
    