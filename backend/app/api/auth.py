from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from .. import schemas, services
from ..database.database import get_db

router = APIRouter()

@router.post("/signup", response_model=schemas.UserOut, status_code=status.HTTP_201_CREATED)
def signup_user(user: schemas.UserCreate, db: Session = Depends(get_db)):
    db_user = services.get_user_by_email(db, email=user.email)
    if db_user:
        raise HTTPException(status_code=400, detail="Email already registered")
    return services.create_user(db=db, user=user)

@router.post("/signin", response_model=schemas.UserOut)
def signin_user(user: schemas.UserLogin, db: Session = Depends(get_db)):
    db_user = services.get_user_by_email(db, email=user.email)
    if not db_user or not services.verify_password(user.password, db_user.hashed_password):
        raise HTTPException(status_code=400, detail="Incorrect email or password")
    return db_user
