

/*-----------------------------------------------------------------------------------
    https://ronnyml.wordpress.com/2009/07/04/listas-enlazadas-clase-lista-en-c/
------------------------------------------------------------------------------------*/
#include "lista.h"
 
using namespace std;
 
// Constructor por defecto
template<typename T>
List<T>::List()
{
    m_num_nodes = 0;
    m_head = NULL;
}

//numero de nodos
template<typename T>
int List<T>::getnum_nod(){
    return m_num_nodes;
}
 
// Insertar al inicio
template<typename T>
void List<T>::add_head(T data_)
{
    Node<T> *new_node = new Node<T> (data_);
    Node<T> *temp = m_head;
 
    if (!m_head) {
        m_head = new_node;
    } else {
        new_node->next = m_head;
        m_head = new_node;
 
        while (temp) {
            temp = temp->next;
        }
    }
    m_num_nodes++;
}
 
// Insertar al final
template<typename T>
void List<T>::add_end(T data_)
{
    Node<T> *new_node = new Node<T> (data_);
    Node<T> *temp = m_head;
 
    if (!m_head) {
        m_head = new_node;
    } else {
        while (temp->next != NULL) {
            temp = temp->next;
        }
        temp->next = new_node;
    }
    m_num_nodes++;
}
 
// Eliminar todos los nodos
template<typename T>
void List<T>::del_all()
{
    m_head->delete_all();
    m_head = 0;
}
 
// Eliminar por data del nodo
template<typename T>
void List<T>::del_by_data(T data_)
{
    Node<T> *temp = m_head;
    Node<T> *temp1 = m_head->next;
 
    int cont = 0;
 
    if (m_head->data == data_) {
        m_head = temp->next;
    } else {
        while (temp1) {
            if (temp1->data == data_) {
                Node<T> *aux_node = temp1;
                temp->next = temp1->next;
                delete aux_node;
                cont++;
                m_num_nodes--;
            }
            temp = temp->next;
            temp1 = temp1->next;
        }
    }
 
    if (cont == 0) {
        cout << "No existe el dato " << endl;
    }
}
 
// Eliminar por posición del nodo
template<typename T>
void List<T>::del_by_position(int pos)
{
    Node<T> *temp = m_head;
    Node<T> *temp1 = temp->next;
 
    if (pos < 1 || pos > m_num_nodes) {
        cout << "Fuera de rango " << endl;
    } else if (pos == 1) {
        m_head = temp->next;
    } else {
        for (int i = 2; i <= pos; i++) {
            if (i == pos) {
                Node<T> *aux_node = temp1;
                temp->next = temp1->next;
                delete aux_node;
                m_num_nodes--;
            }
            temp = temp->next;
            temp1 = temp1->next;
        }
    }
}
 
// Usado por el método intersección
template<typename T>
void insert_sort(T a[], int size)
{
    T temp;
    for (int i = 0; i < size; i++) {
        for (int j = i-1; j>= 0 && a[j+1] < a[j]; j--) {
            temp = a[j+1];
            a[j+1] = a[j];
            a[j] = temp;
        }
    }
}
 
// Imprimir la Lista
template<typename T>
void List<T>::print()
{
    Node<T> *temp = m_head;
    if (!m_head) {
        cout << "La Lista está vacía " << endl;
    } else {
        while (temp) {
            temp->print();
            if (!temp->next) cout << "NULL";
                temp = temp->next;
        }
  }
  cout << endl << endl;
}
 
// Buscar el dato de un nodo
template<typename T>
void List<T>::search(T data_)
{
    Node<T> *temp = m_head;
    int cont = 1;
    int cont2 = 0;
 
    while (temp) {
        if (temp->data == data_) {
            cout << "El dato se encuentra en la posición: " << cont << endl;
            cont2++;
        }
        temp = temp->next;
        cont++;
    }
 
    if (cont2 == 0) {
        cout << "No existe el dato " << endl;
    }
    cout << endl << endl;
}

// Buscar el dato de un nodo por posicion
template<typename T>
Node<T> List<T>::search_by_pos(int data_)
{
    Node<T> *temp = m_head;
    int contador = 0;
    while (temp) {
        if (contador == data_) {
            // cout << "El dato se encuentra en la posición: " << cont << endl;
            // cont2++;
            return *temp;
        }else{
            temp = temp->next;
        }
        contador++;
    }
    return *temp;
}
 
// Ordenar de manera ascendente
template<typename T>
void List<T>::sort()
{
    T temp_data;
    Node<T> *aux_node = m_head;
    Node<T> *temp = aux_node;
 
    while (aux_node) {
        temp = aux_node;
 
        while (temp->next) {
            temp = temp->next;
 
            if (aux_node->data > temp->data) {
                temp_data = aux_node->data;
                aux_node->data = temp->data;
                temp->data = temp_data;
            }
        }
 
        aux_node = aux_node->next;
    }
}
 
template<typename T>
List<T>::~List() {}