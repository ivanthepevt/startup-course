# Startup Foundations (Trainbase Course)

This is a Trainbase-compatible course.

## How to use

```python
!pip install -q git+https://github.com/ivanthepevt/trainbase.git

from trainbase import learn

course = learn("https://github.com/ivanthepevt/startup-course")

course.show_agenda()
course.play("chapter1")
````


---

# 🚀 Cách test ngay trên Colab

```python
!pip install -q git+https://github.com/ivanthepevt/trainbase.git
````

```python
from trainbase import learn

course = learn("https://github.com/ivanthepevt/startup-course")

course.show_agenda()
```

```python
course.play("chapter1")
```
