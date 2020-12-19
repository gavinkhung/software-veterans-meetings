var button = document.querySelector("#todo-button");
var input = document.querySelector("#todo-input");
var list = document.querySelector("#todo-list");

button.onclick = function(){
    console.log(input.value);

    var inputValue = input.value;
    input.value = "";

    var newTodo = document.createElement('li');
    newTodo.textContent = inputValue;

    list.appendChild(newTodo);
}