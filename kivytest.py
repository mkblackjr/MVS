from kivy.app import App
from kivy.uix.label import Label

class MousePos(App):
    def build(self):
        from kivy.core.window import Window
        self.label = Label()
        Window.bind(mouse_pos=lambda w, p: setattr(self.label, 'text', str(p)))
        return self.label

if __name__ == '__main__':
    MousePos().run()