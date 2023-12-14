from demo_python_pkg.person_node import PersonNode

class WriterNode(PersonNode):
    def __init__(self, name: str, age: int, book: str) -> None:
        super().__init__(name, age)
        print('WriterNode 的 __init__ 函数被调用了')
        self.book = book

def main():
    node = WriterNode('法外狂徒张三', 18, '论快速入狱')
    node.eat('鱼香肉丝')
