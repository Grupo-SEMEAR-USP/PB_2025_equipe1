# gui_app.py
import tkinter as tk
from tkinter import scrolledtext
import threading
import queue
import serial_com  # Importa o seu arquivo de comunicação

class SerialGuiApp:
    def __init__(self, root_window):
        self.root = root_window
        self.root.title("Terminal de Comunicação Serial")
        self.root.configure(bg='#282c34') # Fundo geral um pouco mais claro

        # Fila para comunicação segura entre a thread da serial e a GUI
        self.serial_queue = queue.Queue()

        # --- CONFIGURAÇÃO DA INTERFACE ---

        # 1. Área grande e preta para dados recebidos
        self.received_data_text = scrolledtext.ScrolledText(
            self.root, 
            wrap=tk.WORD, 
            bg='black', 
            fg='lime green', # Cor do texto tipo "terminal"
            font=("Consolas", 11),
            state='disabled' # Começa desabilitado para escrita
        )
        self.received_data_text.pack(padx=10, pady=10, expand=True, fill='both')

        # Frame para agrupar a caixa de entrada e o botão
        input_frame = tk.Frame(self.root, bg='#282c34')
        input_frame.pack(padx=10, pady=(0, 10), fill='x')

        # 2. Área retangular preta para escrever dados
        self.send_data_entry = tk.Entry(
            input_frame, 
            bg='black', 
            fg='white', 
            insertbackground='white', # Cor do cursor de digitação
            font=("Consolas", 11),
            width=50
        )
        self.send_data_entry.pack(side='left', expand=True, fill='x', ipady=5)
        # Permite enviar dados pressionando a tecla Enter
        self.send_data_entry.bind("<Return>", self.send_data_event)

        # 3. Botão para enviar
        self.send_button = tk.Button(
            input_frame, 
            text="Enviar", 
            command=self.send_data_event,
            bg='#61afef', # Azul
            fg='white',
            font=("Arial", 10, "bold")
        )
        self.send_button.pack(side='left', padx=(10, 0))

        # --- INICIALIZAÇÃO DA LÓGICA ---

        # Inicia a thread que fica lendo a porta serial em segundo plano
        self.start_serial_thread()

        # Inicia o processo que verifica a fila por novas mensagens
        self.process_serial_queue()

    def send_data_event(self, event=None):
        """Pega o texto da caixa de entrada e o envia pela serial."""
        data_to_send = self.send_data_entry.get()
        if data_to_send:
            serial_com.write_serial(data_to_send + "\n") # Adiciona \n por padrão
            self.display_message(f"ENVIADO >> {data_to_send}\n", "cyan")
            self.send_data_entry.delete(0, tk.END)

    def display_message(self, message, color="lime green"):
        """Exibe uma mensagem na área de texto de recebimento."""
        self.received_data_text.configure(state='normal') # Habilita para escrita
        self.received_data_text.insert(tk.END, message)
        self.received_data_text.configure(state='disabled') # Desabilita novamente
        self.received_data_text.see(tk.END) # Rola para o final automaticamente

    def serial_thread_worker(self):
        """Função que roda na thread separada, lendo a serial constantemente."""
        while True:
            # A função read_serial() pode ser bloqueante, por isso está em uma thread
            received_message = serial_com.read_serial()
            if received_message:
                # Coloca a mensagem na fila para a GUI processar com segurança
                self.serial_queue.put(received_message)

    def start_serial_thread(self):
        """Cria e inicia a thread de leitura da serial."""
        serial_thread = threading.Thread(target=self.serial_thread_worker, daemon=True)
        serial_thread.start()

    def process_serial_queue(self):
        """Verifica se há novas mensagens na fila e as exibe na GUI."""
        try:
            # Pega uma mensagem da fila sem bloquear
            message = self.serial_queue.get_nowait()
            self.display_message(f"RECEBIDO << {message}\n")
        except queue.Empty:
            # Se a fila estiver vazia, não faz nada
            pass
        finally:
            # Agenda esta mesma função para rodar novamente em 100ms
            self.root.after(10, self.process_serial_queue)


if __name__ == "__main__":
    # Cria a janela principal da aplicação
    root = tk.Tk()
    # Cria a instância da nossa classe de aplicação
    app = SerialGuiApp(root)
    # Inicia o loop principal da interface, que a mantém rodando
    root.mainloop()