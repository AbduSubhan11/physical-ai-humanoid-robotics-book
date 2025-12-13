from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer, OAuth2PasswordRequestForm
from jose import JWTError, jwt
from datetime import datetime, timedelta
from typing import Optional
from ... import schemas
from ...config import settings

router = APIRouter()

oauth2_scheme = OAuth2PasswordBearer(tokenUrl="token")

@router.post("/register", response_model=schemas.User)
async def register_user(user: schemas.UserCreate):
    """
    Register a new user
    """
    # TODO: Implement actual user registration logic
    # For now, return a mock response
    return schemas.User(
        id="mock-user-id",
        email=user.email,
        username=user.username or user.email.split("@")[0],
        is_active=True,
        created_at=datetime.utcnow(),
        updated_at=datetime.utcnow()
    )

@router.post("/login")
async def login_user(form_data: OAuth2PasswordRequestForm = Depends()):
    """
    Authenticate user and return access token
    """
    # TODO: Implement actual authentication logic
    # For now, return a mock token
    return {
        "access_token": "mock-jwt-token",
        "token_type": "bearer"
    }

def create_access_token(data: dict, expires_delta: Optional[timedelta] = None):
    """
    Create JWT access token
    """
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=15)

    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, settings.SECRET_KEY, algorithm=settings.ALGORITHM)
    return encoded_jwt

async def get_current_user(token: str = Depends(oauth2_scheme)):
    """
    Get current user from JWT token
    """
    credentials_exception = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Could not validate credentials",
        headers={"WWW-Authenticate": "Bearer"},
    )
    try:
        payload = jwt.decode(token, settings.SECRET_KEY, algorithms=[settings.ALGORITHM])
        username: str = payload.get("sub")
        if username is None:
            raise credentials_exception
    except JWTError:
        raise credentials_exception

    # TODO: Fetch user from database
    # For now, return mock user
    return schemas.User(
        id="mock-user-id",
        email="user@example.com",
        username=username,
        is_active=True,
        created_at=datetime.utcnow(),
        updated_at=datetime.utcnow()
    )