import pytest
from mock import Mock
from choirbot.communicators.callback_groups import AuthorizationCallbackGroup

@pytest.fixture
def cbgroup():
    a = AuthorizationCallbackGroup()
    b = Mock()
    return (a,b)

def test_authorized(cbgroup):
    cb, ent = cbgroup
    cb.give_authorization()
    assert cb.can_execute(ent) == True

def test_unauthorized(cbgroup):
    cb, ent = cbgroup
    cb.draw_authorization()
    assert cb.can_execute(ent) == False

def test_authorized_execution(cbgroup):
    cb, ent = cbgroup
    cb.give_authorization()
    assert cb.beginning_execution(ent) == True
    assert cb.can_execute(ent) == False

def test_unauthorized_execution(cbgroup):
    cb, ent = cbgroup
    cb.draw_authorization()
    assert cb.beginning_execution(ent) == False