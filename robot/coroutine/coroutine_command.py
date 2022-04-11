from functools import wraps
from typing import Any, Callable, Generator, Iterable, List, Union, Optional
from commands2 import CommandBase, Subsystem
import inspect

def ensure_generator_function(func: Union[Callable[..., None], Callable[..., Generator[None, None, None]]]) -> Callable[..., Generator[None, None, None]]:
	if inspect.isgeneratorfunction(func):
		return func # type: ignore

	@wraps(func)
	def wrapper(*args, **kwargs):
		func(*args, **kwargs)
		yield

	return wrapper


class CoroutineCommand(CommandBase):
	coroutine: Union[Callable[..., None], Callable[..., Generator[None, None, None]], Generator[None, None, None]]
	is_finished: bool

	def __init__(self, coroutine: Union[Callable[..., None], Callable[..., Generator[None, None, None]], Generator[None, None, None]], requirements: Optional[List[Subsystem]] = None) -> None:
		super().__init__()
		self.coroutineable = coroutine
		self.coroutine = None
		self.is_finished = False

		if requirements is not None:
			self.addRequirements(requirements)

	def initialize(self) -> None:
		self.coroutine = ensure_generator_function(self.coroutineable)()
		self.is_finished = False

	def execute(self):
		# __import__("code").interact(local={**locals(), **globals()})
		# print(self.coroutine)
		try:
			if not self.is_finished:
				if not inspect.isgenerator(self.coroutine):
					# __import__("code").interact(local={**locals(), **globals()})
					raise TypeError("This command was not properly initialized")
				next(self.coroutine)
		except StopIteration:
			self.is_finished = True

	def isFinished(self):
		return self.is_finished

	def end(self, interrupted: bool):
		if not self.is_finished:
			self.coroutine.close()

	def __call__(self, *args, **kwargs) -> "CoroutineCommand":
		if inspect.isgenerator(self.coroutine):
			return self

		return CoroutineCommand(ensure_generator_function(self.coroutine)(*args, **kwargs)) # type: ignore

def commandify(func):
	@wraps(func)
	def wrapper(*args, **kwargs):
		gen = func(*args, **kwargs)

		class C(CommandBase):
			def __init__(self) -> None:
				super().__init__()
				self.is_finished = False
				r: "Robot" = args[0]
				self.addRequirements(r.subsystems)
			def execute(self) -> None:
				try:
					next(gen)
				except StopIteration:
					self.is_finished = True
			def isFinished(self) -> bool:
				return self.is_finished
		return C()
	return wrapper

